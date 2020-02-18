/**
 * Generates an OSM PBF file from a simple ascii map and a couple of dictionaries
 * of key/val properties.
 *
 * Any A-Za-z0-9 character on the map defines a node with a coordinate.
 * Other characters are ignored, but can be used to help visualization
 *
 * The wayprops define how nodes are joined together (the way name is a simple string of
 * single character node names).  Tags can be attached too.
 *
 * Simple example program:
 *
const std::string asciimap = R"(
  A-----B-----C
  |
  D-----E-----F
)";

using vtm = valhalla::test::mapgen;

const vtm::props wayprops = {{"ABC", {{"highway", "primary"}}},
                             {"DEF", {{"highway", "primary"}}},
                             {"AD", {{"highway", "primary"}}}};

const vtm::props nodeprops = {{"A", {{"barrier", "true"}}}};
const vtm::relations rels = {
    {{vtm::member{vtm::way, "ABC", "from"},
     {vtm::member{vtm::way, "DEF", "to"}},
     {vtm::member{vtm::way, "AD", "via"}}},
    {{"type", "restriction"},
     {"restriction", "no_right_turn"}}}
};

int main() {

  auto node_coords = vtm::map_to_coordinates(map, 100);
  vtm::build_pbf(node_coords, ways, nodeprops, rels, "test.pbf");

  return 0;
}
**************************************************************/
#include <exception>
#include <iostream>
#include <regex>
#include <string>
#include <unordered_map>
#include <unordered_set>

// Allow any format of output files (XML, PBF, ...)
#include <osmium/io/pbf_output.hpp>

// We want to use the builder interface
#include <osmium/builder/attr.hpp>
#include <osmium/builder/osm_object_builder.hpp>

namespace valhalla {
namespace test {
namespace mapgen {

using keyval = std::unordered_map<std::string, std::string>;
using props = std::unordered_map<std::string, keyval>;
struct lonlat {
  double lon;
  double lat;
  int id = -1;
};
enum _member_type { node, way };
struct member {
  _member_type type;
  std::string ref;
  std::string role;
};
struct relation {
  std::vector<member> members;
  keyval tags;
};

using relations = std::vector<relation>;
using nodemap = std::unordered_map<std::string, lonlat>;

std::vector<std::string> splitter(const std::string in_pattern, const std::string& content) {
  std::vector<std::string> split_content;
  std::regex pattern(in_pattern);
  std::copy(std::sregex_token_iterator(content.begin(), content.end(), pattern, -1),
            std::sregex_token_iterator(), std::back_inserter(split_content));
  return split_content;
}

void ltrim(std::string& s) {
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch) { return !std::isspace(ch); }));
}
void rtrim(std::string& s) {
  s.erase(std::find_if(s.rbegin(), s.rend(), [](int ch) { return !std::isspace(ch); }).base(),
          s.end());
}

std::string trim(std::string s) {
  ltrim(s);
  rtrim(s);
  return s;
}

/**
 * Given a string that's an "ASCII map", will decide on coordinates
 * for the nodes drawn on the grid.
 *
 * @returns a dictionary of node IDs to lon/lat values
 */
inline nodemap map_to_coordinates(const std::string& map,
                                  const double gridsize_metres,
                                  const lonlat& topleft = {0, 0}) {

  // Gridsize is in meters per character

  const double earth_mean_radius = 6371008.8;
  const double DEGREE_TO_RAD = 0.017453292519943295769236907684886;
  const double metres_to_degrees = 1 / (DEGREE_TO_RAD * earth_mean_radius);
  const double grid_to_degree = gridsize_metres * metres_to_degrees;

  // Split string into lines
  // Strip whitespace lines, if they exist
  // Strip common leading whitespace
  // Decide locations for nodes based on remaining grid

  // Split string into lines
  auto lines = splitter("\n", map);
  if (lines.empty())
    return {};

  // Remove the leading whitespace lines, if they exist
  while (trim(lines.front()).empty()) {
    lines.erase(lines.begin());
  }

  // Find out the min whitespace on each line, then remove that from each line
  long min_whitespace = std::numeric_limits<long>::max();
  for (const auto& line : lines) {
    // Skip blank lines, as they might have no space at all, but shouldn't
    // be allowed to affect positioning
    if (trim(line).empty())
      continue;
    auto pos =
        std::find_if(line.begin(), line.end(), [](const auto& ch) { return !std::isspace(ch); });
    min_whitespace = std::min(min_whitespace, std::distance(line.begin(), pos));
    if (min_whitespace == 0) // No need to continue if something is up against the left
      break;
  }

  // In-place remove leading whitespace from each line
  for (auto& line : lines) {
    // This must be a blank line or something
    if (line.size() < min_whitespace)
      continue;
    line.erase(line.begin(), line.begin() + min_whitespace);
  }

  // TODO: the way this projects coordinates onto the sphere could do with some work.
  //       it's not always going to be sensible to lay a grid down onto a sphere
  nodemap result;
  for (int y = 0; y < lines.size(); y++) {
    for (int x = 0; x < lines[y].size(); x++) {
      auto ch = lines[y][x];
      // Only do A-Za-z0-9 for nodes - all other things are ignored
      if (std::isalnum(ch)) {
        // Always project west, then south, for consistency
        double lon = topleft.lon + grid_to_degree * x;
        double lat = topleft.lat - grid_to_degree * y;
        result.insert({std::string(1, ch), {lon, lat}});
      }
    }
  }

  return result;
}

/**
 * Given a map of node locations, ways, node properties and relations, will
 * generate an OSM compatible PBF file, suitable for loading into Valhalla
 */
inline void build_pbf(nodemap& node_locations,
                      const props& ways,
                      const props& nodes,
                      const relations& relations,
                      const std::string& filename,
                      const int initial_osm_id = 0) {

  const size_t initial_buffer_size = 10000;
  osmium::memory::Buffer buffer{initial_buffer_size, osmium::memory::Buffer::auto_grow::yes};

  std::unordered_set<std::string> used_nodes;
  for (const auto& way : ways) {
    for (const auto& ch : way.first) {
      used_nodes.insert(std::string(1, ch));
    }
  }
  for (const auto& node : nodes) {
    for (const auto& ch : node.first) {
      used_nodes.insert(std::string(1, ch));
    }
  }
  for (const auto& relation : relations) {
    for (const auto& member : relation.members) {
      if (member.type == node) {
        used_nodes.insert(member.ref);
      }
    }
  }

  for (auto& used_node : used_nodes) {
    if (node_locations.count(used_node) == 0) {
      throw std::runtime_error("Node " + used_node + " was referred to but was not in the ASCII map");
    }
  }

  std::unordered_map<std::string, int> node_id_map;
  int id = 0;
  for (auto& loc : node_locations) {
    node_id_map[loc.first] = id++;
  }
  int osm_id = initial_osm_id;
  for (auto& loc : node_locations) {
    if (used_nodes.count(loc.first) > 0) {
      loc.second.id = osm_id++;

      std::vector<std::pair<std::string, std::string>> tags;

      if (nodes.count(loc.first) == 0) {
        tags.push_back({"name", loc.first});
      } else {
        auto otags = nodes.at(loc.first);
        if (otags.count("name") == 0) {
          tags.push_back({"name", loc.first});
        }
        for (const auto& keyval : otags) {
          tags.push_back({keyval.first, keyval.second});
        }
      }

      osmium::builder::add_node(buffer, osmium::builder::attr::_id(loc.second.id),
                                osmium::builder::attr::_version(1),
                                osmium::builder::attr::_timestamp(std::time(nullptr)),
                                osmium::builder::attr::_location(
                                    osmium::Location{loc.second.lon, loc.second.lat}),
                                osmium::builder::attr::_tags(tags));
    }
  }

  std::unordered_map<std::string, int> way_osm_id_map;
  for (const auto& way : ways) {
    way_osm_id_map[way.first] = osm_id;
    std::vector<int> nodeids;
    for (const auto& ch : way.first) {
      nodeids.push_back(node_locations[std::string(1, ch)].id);
    }
    std::vector<std::pair<std::string, std::string>> tags;
    if (way.second.count("name") == 0) {
      tags.push_back({"name", way.first});
    }
    for (const auto& keyval : way.second) {
      tags.push_back({keyval.first, keyval.second});
    }
    osmium::builder::add_way(buffer, osmium::builder::attr::_id(osm_id++),
                             osmium::builder::attr::_version(1),
                             osmium::builder::attr::_timestamp(std::time(nullptr)),
                             osmium::builder::attr::_nodes(nodeids),
                             osmium::builder::attr::_tags(tags));
  }

  for (const auto& relation : relations) {

    std::vector<osmium::builder::attr::member_type> members;

    for (const auto& member : relation.members) {
      if (member.type == node) {
        members.push_back({osmium::item_type::node, node_locations[member.ref].id});
      } else {
        if (way_osm_id_map.count(member.ref) == 0) {
          throw std::runtime_error("Relation member refers to an undefined way " + member.ref);
        }
        members.push_back({osmium::item_type::way, way_osm_id_map[member.ref], member.role.c_str()});
      }
    }

    std::vector<std::pair<std::string, std::string>> tags;
    for (const auto& tag : relation.tags) {
      tags.push_back({tag.first, tag.second});
    }

    osmium::builder::add_relation(buffer, osmium::builder::attr::_id(osm_id++),
                                  osmium::builder::attr::_version(1),
                                  osmium::builder::attr::_timestamp(std::time(nullptr)),
                                  osmium::builder::attr::_members(members),
                                  osmium::builder::attr::_tags(tags));
  }

  // Create header and set generator.
  osmium::io::Header header;
  header.set("generator", "valhalla-test-creator");

  osmium::io::File output_file{filename, "pbf"};

  // Initialize Writer using the header from above and tell it that it
  // is allowed to overwrite a possibly existing file.
  osmium::io::Writer writer{output_file, header, osmium::io::overwrite::allow};

  // Write out the contents of the output buffer.
  writer(std::move(buffer));

  // Explicitly close the writer. Will throw an exception if there is
  // a problem. If you wait for the destructor to close the writer, you
  // will not notice the problem, because destructors must not throw.
  writer.close();
}

} // namespace mapgen
} // namespace test
} // namespace valhalla