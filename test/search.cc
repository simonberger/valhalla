#include "loki/search.h"
#include <cstdint>

#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <unordered_set>

#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/location.h"
#include "baldr/pathlocation.h"
#include "baldr/tilehierarchy.h"
#include "midgard/pointll.h"
#include "midgard/vector2.h"
#include "sif/autocost.h"
#include "sif/nocost.h"

#include "gurka/gurka.h"
#include "test.h"

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::loki;
namespace vs = valhalla::sif;

#include "mjolnir/directededgebuilder.h"
#include "mjolnir/graphtilebuilder.h"

namespace {

std::unordered_map<std::string, midgard::PointLL> node_locations;
const std::string map1 = R"(
    B
    |\
  1 | \ 0
    |  \
  2 |   \ 7
    |    \
    A-3-8-D
    |    /
  4 |   / 9
    |  /
  5 | / 6
    |/
    C
)";
// const gurka::ways ways1 = {
//    {"AB", {{"highway", "residential"}, {"foot", "yes"}}},
//    {"AC", {{"highway", "residential"}, {"foot", "yes"}}},
//    {"AD", {{"highway", "residential"}, {"foot", "yes"}}},
//    {"BD", {{"highway", "residential"}, {"foot", "yes"}}},
//    {"CD", {{"highway", "residential"}, {"foot", "yes"}}},
//};
const gurka::ways ways1 = {
    {"BADB", {{"highway", "motorway"}, {"foot", "yes"}}},
    {"ACD", {{"highway", "motorway"}, {"foot", "yes"}}},
};

const std::string tile_dir = "test/data/search_tiles";
const std::string config_file = "test/test_trivial_path";

void write_config(const std::string& filename,
                  const std::string& tile_dir = "test/data/search_tiles") {
  std::ofstream file;
  try {
    file.open(filename, std::ios_base::trunc);
    file << "{ \
      \"mjolnir\": { \
      \"concurrency\": 1, \
       \"tile_dir\": \"" +
                tile_dir + "\", \
        \"admin\": \"" VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite\", \
         \"timezone\": \"" VALHALLA_SOURCE_DIR "test/data/not_needed.sqlite\" \
      } \
    }";
  } catch (...) {}
  file.close();
}
const GraphId tile_id = TileHierarchy::GetGraphId({.125, .125}, 2);
PointLL base_ll = TileHierarchy::get_tiling(tile_id.level()).Base(tile_id.tileid());
std::pair<GraphId, PointLL> b({tile_id.tileid(), tile_id.level(), 0}, {.01, .2});
std::pair<GraphId, PointLL> a({tile_id.tileid(), tile_id.level(), 1}, {.01, .1});
std::pair<GraphId, PointLL> c({tile_id.tileid(), tile_id.level(), 2}, {.01, .01});
std::pair<GraphId, PointLL> d({tile_id.tileid(), tile_id.level(), 3}, {.2, .1});

void clean_tiles() {
  if (boost::filesystem::is_directory(tile_dir)) {
    boost::filesystem::remove_all(tile_dir);
  }
}

void make_tile() {

  if (boost::filesystem::exists(tile_dir))
    boost::filesystem::remove_all(tile_dir);

  boost::filesystem::create_directories(tile_dir);

  boost::property_tree::ptree conf;
  write_config(config_file, tile_dir);
  rapidjson::read_json(config_file, conf);

  // We don't want these in our test tile
  conf.put("mjolnir.hierarchy", false);
  conf.put("mjolnir.shortcuts", false);

  const double gridsize = 666;

  {
    // Build the maps from the ASCII diagrams, and extract the generated lon,lat values
    auto nodemap = gurka::detail::map_to_coordinates(map1, gridsize, {0.01, 0.2});
    const int initial_osm_id = 0;
    gurka::detail::build_pbf(nodemap, ways1, {}, {}, tile_dir + "/map1.pbf", initial_osm_id);
    for (const auto& n : nodemap)
      node_locations[n.first] = n.second;
  }

  {
    constexpr bool release_osmpbf_memory = false;
    mjolnir::build_tile_set(conf, {tile_dir + "/map1.pbf"}, mjolnir::BuildStage::kInitialize,
                            mjolnir::BuildStage::kValidate, release_osmpbf_memory);
    /** Set the freeflow and constrained flow speeds manually on all edges */
    mjolnir::GraphTileBuilder tile_builder(tile_dir, tile_id, false);
    std::vector<DirectedEdge> directededges;
    directededges.reserve(tile_builder.header()->directededgecount());
    for (uint32_t j = 0; j < tile_builder.header()->directededgecount(); ++j) {
      // skip edges for which we dont have speed data
      DirectedEdge& directededge = tile_builder.directededge(j);
      directededge.set_free_flow_speed(100);
      directededge.set_constrained_flow_speed(10);
      directededge.set_forwardaccess(baldr::kAllAccess);
      directededge.set_reverseaccess(baldr::kAllAccess);
      directededges.emplace_back(std::move(directededge));
    }
    tile_builder.UpdatePredictedSpeeds(directededges);
  }

  GraphTile tile(tile_dir, tile_id);
  ASSERT_EQ(tile.FileSuffix(tile_id, false), std::string("2/000/519/120.gph"))
      << "Tile ID didn't match the expected filename";

  ASSERT_PRED1(filesystem::exists, tile_dir + "/" + tile.FileSuffix(tile_id, false))
      << "Expected tile file didn't show up on disk - are the fixtures in the right location?";
}

std::shared_ptr<vs::DynamicCost> create_costing() {
  valhalla::Options options;
  const rapidjson::Document doc;
  vs::ParseAutoCostOptions(doc, "/costing_options/auto", options.add_costing_options());
  vs::ParseAutoShorterCostOptions(doc, "/costing_options/auto_shorter",
                                  options.add_costing_options());
  options.add_costing_options();
  // return nullptr;
  // return std::make_shared<vs::DynamicCost>(valhalla::Costing::auto_, options);
  // return vs::CreateAutoCost(valhalla::Costing::auto_, options);
  return vs::CreateNoCost(valhalla::Costing::auto_, options);
}

void search(valhalla::baldr::Location location,
            bool expected_node,
            const valhalla::midgard::PointLL& expected_point,
            const std::vector<PathLocation::PathEdge>& expected_edges,
            bool exact = false) {
  // make the config file
  boost::property_tree::ptree conf;
  conf.put("tile_dir", tile_dir);
  valhalla::baldr::GraphReader reader(conf);

  // send it to pbf and back just in case something is wrong with that conversion
  valhalla::Location pbf;
  PathLocation::toPBF(location, &pbf, reader);
  location = PathLocation::fromPBF(pbf);
  const auto costing = create_costing();

  const auto results = Search({location}, reader, costing);
  std::cout << "Got results.size() " << results.size() << std::endl;
  const auto& p = results.at(location);

  ASSERT_EQ((p.edges.front().begin_node() || p.edges.front().end_node()), expected_node)
      << p.edges.front().begin_node() << ":" << p.edges.front().end_node()
      << (expected_node ? " Should've snapped to node" : " Shouldn't've snapped to node");

  ASSERT_TRUE(p.edges.size()) << "Didn't find any node/edges";
  ASSERT_TRUE(p.edges.front().projected.ApproximatelyEqual(expected_point)) << "Found wrong point";

  valhalla::baldr::PathLocation answer(location);
  for (const auto& expected_edge : expected_edges) {
    answer.edges.emplace_back(
        PathLocation::PathEdge{expected_edge.id, expected_edge.percent_along, expected_point,
                               expected_point.Distance(location.latlng_), expected_edge.sos});
  }
  // note that this just checks that p has the edges that answer has
  // p can have more edges than answer has and that wont fail this check!
  ASSERT_TRUE(answer.shares_edges(p)) << "Did not find expected edges";
  // if you want to enforce that the result didnt have more then expected
  if (exact) {
    ASSERT_EQ(answer.edges.size(), p.edges.size()) << "Got more edges than expected";
  }
}

void search(valhalla::baldr::Location location, size_t result_count, int reachability) {

  // make the config file
  boost::property_tree::ptree conf;
  conf.put("tile_dir", tile_dir);
  valhalla::baldr::GraphReader reader(conf);

  // send it to pbf and back just in case something is wrong with that conversion
  valhalla::Location pbf;
  PathLocation::toPBF(location, &pbf, reader);
  location = PathLocation::fromPBF(pbf);
  const auto costing = create_costing();

  const auto results = Search({location}, reader, costing);
  if (results.empty() && result_count == 0)
    return;

  const auto& path = results.at(location);

  ASSERT_EQ(path.edges.size(), result_count) << "Wrong number of edges";
  for (const auto& edge : path.edges) {
    ASSERT_EQ(edge.outbound_reach, reachability);
    ASSERT_EQ(edge.inbound_reach, reachability); // TODO
  }
}

TEST(Search, test_edge_search) {
  auto t = a.first.tileid();
  auto l = a.first.level();
  using S = PathLocation::SideOfStreet;
  using PE = PathLocation::PathEdge;
  using ST = baldr::Location::StopType;
  using PS = baldr::Location::PreferredSide;

  // snap to node searches
  search({a.second}, true, a.second,
         {PE{{t, l, 2}, 0, a.second, 0, S::NONE}, PE{{t, l, 3}, 0, a.second, 0, S::NONE},
          PE{{t, l, 4}, 0, a.second, 0, S::NONE}});
  search({d.second}, true, d.second,
         {PE{{t, l, 7}, 0, d.second, 0, S::NONE}, PE{{t, l, 8}, 0, d.second, 0, S::NONE},
          PE{{t, l, 9}, 0, d.second, 0, S::NONE}});
  // snap to end of edge degenerates to node searches
  search({{d.second.first + .049, d.second.second}}, true, d.second,
         {
             PE{{t, l, 7}, 0, d.second, 0, S::NONE}, PE{{t, l, 8}, 0, d.second, 0, S::NONE},
             PE{{t, l, 9}, 0, d.second, 0, S::NONE}, // leaving edges
             PE{{t, l, 0}, 1, d.second, 0, S::NONE}, PE{{t, l, 3}, 1, d.second, 0, S::NONE},
             PE{{t, l, 6}, 1, d.second, 0, S::NONE} // arriving edges
         });

  // regression test for #2023, displace the point beyond search_cutoff but within node_snap_tolerance
  baldr::Location near_node{a.second};
  near_node.latlng_.second += 0.00001; // 1.11 meters
  near_node.search_cutoff_ = 1.0;
  near_node.node_snap_tolerance_ = 2.0;
  search(near_node, true, a.second,
         {PE{{t, l, 2}, 0, a.second, 0, S::NONE}, PE{{t, l, 3}, 0, a.second, 0, S::NONE},
          PE{{t, l, 4}, 0, a.second, 0, S::NONE}, PE{{t, l, 1}, 1, a.second, 0, S::NONE},
          PE{{t, l, 8}, 1, a.second, 0, S::NONE}, PE{{t, l, 5}, 1, a.second, 0, S::NONE}},
         true);

  // snap to node as through location should be all edges
  baldr::Location x{a.second, baldr::Location::StopType::THROUGH};
  search(x, true, a.second,
         {PE{{t, l, 2}, 0, a.second, 0, S::NONE}, PE{{t, l, 3}, 0, a.second, 0, S::NONE},
          PE{{t, l, 4}, 0, a.second, 0, S::NONE}, PE{{t, l, 1}, 1, a.second, 0, S::NONE},
          PE{{t, l, 8}, 1, a.second, 0, S::NONE}, PE{{t, l, 5}, 1, a.second, 0, S::NONE}},
         true);
  // with a heading should just be a single outgoing
  x.heading_ = 180;
  search(x, true, a.second, {PE{{t, l, 4}, 0, a.second, 0, S::NONE}}, true);

  // mid point search
  auto answer = a.second.MidPoint(d.second);
  search({a.second.MidPoint(d.second)}, false, a.second.MidPoint(d.second),
         {PE{{t, l, 3}, .5f, answer, 0, S::NONE}, PE{{t, l, 8}, .5f, answer, 0, S::NONE}});

  // set a point 40% along the edge runs with the shape direction
  answer = a.second.AffineCombination(.6f, .4f, d.second);
  auto ratio = a.second.Distance(answer) / a.second.Distance(d.second);
  x = {answer};
  search(x, false, answer,
         {PE{{t, l, 3}, ratio, answer, 0, S::NONE}, PE{{t, l, 8}, 1.f - ratio, answer, 0, S::NONE}});

  // with heading
  x.heading_ = 90;
  search(x, false, answer, {PE{{t, l, 3}, ratio, answer, 0, S::NONE}});
  x.heading_ = 0;
  search(x, false, answer,
         {PE{{t, l, 3}, ratio, answer, 0, S::NONE}, PE{{t, l, 8}, 1.f - ratio, answer, 0, S::NONE}});
  x.heading_ = 269;
  search(x, false, answer, {PE{{t, l, 8}, 1.f - ratio, answer, 0, S::NONE}});

  // check for side of street by offsetting the test point from the line orthogonally
  auto ortho = (d.second - a.second).GetPerpendicular(true).Normalize() * .01;
  PointLL test{answer.first + ortho.x(), answer.second + ortho.y()};
  search({test}, false, answer,
         {PE{{t, l, 3}, ratio, answer, 0, S::RIGHT}, PE{{t, l, 8}, 1.f - ratio, answer, 0, S::LEFT}});

  // check that the side of street tolerance works
  baldr::Location sst_huge(test);
  sst_huge.street_side_tolerance_ = 2000;
  search(sst_huge, false, answer,
         {PE{{t, l, 3}, ratio, answer, 0, S::NONE}, PE{{t, l, 8}, 1.f - ratio, answer, 0, S::NONE}});

  // we only want opposite driving side, tiles are left hand driving
  search({test, ST::BREAK, 0, 0, 0, PS::OPPOSITE}, false, answer,
         {PE{{t, l, 3}, ratio, answer, 0, S::RIGHT}}, true);

  // we only want same driving side, tiles are left hand driving
  search({test, ST::BREAK, 0, 0, 0, PS::SAME}, false, answer,
         {PE{{t, l, 8}, 1.f - ratio, answer, 0, S::LEFT}}, true);

  // set a point 40% along the edge that runs in reverse of the shape
  answer = b.second.AffineCombination(.6f, .4f, d.second);
  ratio = b.second.Distance(answer) / b.second.Distance(d.second);
  search({answer}, false, answer,
         {PE{{t, l, 0}, ratio, answer, 0, S::NONE}, PE{{t, l, 7}, 1.f - ratio, answer, 0, S::NONE}});

  // check for side of street by offsetting the test point from the line orthogonally
  ortho = (d.second - b.second).GetPerpendicular(false).Normalize() * .01;
  test.Set(answer.first + ortho.x(), answer.second + ortho.y());
  search({test}, false, answer,
         {PE{{t, l, 0}, ratio, answer, 0, S::LEFT}, PE{{t, l, 7}, 1.f - ratio, answer, 0, S::RIGHT}});

  // check that the side of street tolerance works
  sst_huge.latlng_ = test;
  search(sst_huge, false, answer,
         {PE{{t, l, 0}, ratio, answer, 0, S::NONE}, PE{{t, l, 7}, 1.f - ratio, answer, 0, S::NONE}});

  // we only want opposite driving side, tiles are left hand driving
  search({test, ST::BREAK, 0, 0, 0, PS::OPPOSITE}, false, answer,
         {PE{{t, l, 7}, 1.f - ratio, answer, 0, S::RIGHT}}, true);

  // we only want same driving side, tiles are left hand driving
  search({test, ST::BREAK, 0, 0, 0, PS::SAME}, false, answer,
         {PE{{t, l, 0}, ratio, answer, 0, S::LEFT}}, true);

  // TODO: add more tests
}

TEST(Search, test_reachability_radius) {

  // DEBUG
  boost::property_tree::ptree conf;
  conf.put("tile_dir", tile_dir);
  valhalla::baldr::GraphReader reader(conf);

  auto tile = reader.GetGraphTile(tile_id);
  int edges = 0;
  std::cout << "Edges in tile " << std::endl;
  for (auto bin = 0; bin < 25; ++bin) {
    for (auto edge_id : tile->GetBin(bin)) {
      // std::cout << edge_id.id() << ", ";
      auto edge = reader.directededge(edge_id);
      auto node = reader.GetEndNode(edge, tile);

      auto edge_info = tile->edgeinfo(edge->edgeinfo_offset());
      for (auto& name : edge_info.GetNames()) {
        std::cout << name << " ";
      }

      ++edges;
    }
  }
  std::cout << "\nfound " << edges << std::endl;
  ASSERT_EQ(edges, 10);

  auto A = node_locations["A"];
  auto B = node_locations["B"];
  auto D = node_locations["D"];
  PointLL ob(B.lng() - .001f, B.lat() - .01f);
  unsigned int longest = ob.Distance(D);
  unsigned int shortest = ob.Distance(A);

  // LOGLN_WARN("zero everything should be a single closest result");
  // search({ob, baldr::Location::StopType::BREAK, 0, 0, 0}, 2, 0);

  LOGLN_WARN("set radius high to get them all");
  search({B, baldr::Location::StopType::BREAK, 0, 0, longest + 100}, 10, 0);

  // LOGLN_WARN("set radius mid to get just some");
  // search({b.second, baldr::Location::StopType::BREAK, 0, 0, shortest - 100}, 4, 0);

  // LOGLN_WARN("set reachability high to see it gets all nodes reachable");
  // TODO figure out if this is correct. It is good enough for now
  // It is complicated by the fact that u-turn detection and similar never
  // was iimplemented for Isochrone/Dijkstras
  // auto expected_reach = 10;
  // search({ob, Location::StopType::BREAK, 10, 10, 0}, 2, expected_reach);

  // LOGLN_WARN("set reachability right on to see we arent off by one");
  // search({ob, baldr::Location::StopType::BREAK, 4, 4, 0}, 2, 4);

  // LOGLN_WARN("set reachability lower to see we give up early");
  // search({ob, Location::StopType::BREAK, 3, 3, 0}, 2, 3);
}

TEST(Search, test_search_cutoff) {
  // test default cutoff of 35km showing no results
  search({{-77, -77}}, 0, 0);

  // test limits of cut off
  auto t = c.second;
  t.first -= 1;
  t.second -= 1;
  auto dist = t.Distance(c.second);

  // set the cut off just too small and get nothing
  baldr::Location x(t);
  x.search_cutoff_ = dist - 1;
  search(x, 0, 0);

  // set the cut off just big enough and get one node snap result with 4 edges at it
  x.search_cutoff_ = dist + 1;
  search(x, 4, 0);

  // lets try an edge snap
  t = c.second;
  t.first -= .00005;
  t.second += .00005;
  dist = t.Distance({0.01, 0.01005});

  // just out of reach
  x.latlng_ = t;
  x.search_cutoff_ = dist - 1;
  search(x, 0, 0);

  // just within reach
  x.search_cutoff_ = dist + 1;
  search(x, 2, 0);
}

} // namespace

// Setup and tearown will be called only once for the entire suite121
class SearchTestSuiteEnv : public ::testing::Environment {
public:
  void SetUp() override {
    make_tile();
  }
  void TearDown() override {
    // todo: think whether we want to clean tile dir after the test
  }
};

int main(int argc, char* argv[]) {
  testing::AddGlobalTestEnvironment(new SearchTestSuiteEnv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
