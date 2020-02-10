#include "test.h"
#include <cstdint>
#include <fstream>

#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/location.h"
#include "baldr/rapidjson_utils.h"
#include "baldr/tilehierarchy.h"
#include "filesystem.h"
#include "loki/search.h"
#include "loki/worker.h"
#include "midgard/pointll.h"
#include "midgard/vector2.h"
#include "mjolnir/graphbuilder.h"
#include "mjolnir/graphenhancer.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/graphvalidator.h"
#include "mjolnir/pbfgraphparser.h"
#include "odin/directionsbuilder.h"
#include "odin/worker.h"
#include "sif/costconstants.h"
#include "sif/dynamiccost.h"
#include "sif/pedestriancost.h"
#include "thor/astar.h"
#include "thor/attributes_controller.h"
#include "thor/bidirectional_astar.h"
#include "thor/pathalgorithm.h"
#include "thor/timedep.h"
#include "thor/triplegbuilder.h"
#include "thor/worker.h"
#include "tyr/serializers.h"

#include <valhalla/proto/directions.pb.h>
#include <valhalla/proto/options.pb.h>
#include <valhalla/proto/trip.pb.h>

#include <boost/algorithm/string/join.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/property_tree/ptree.hpp>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

namespace bpt = boost::property_tree;

using namespace valhalla;
namespace vm = valhalla::midgard;
namespace vb = valhalla::baldr;
namespace vs = valhalla::sif;
namespace vt = valhalla::thor;
namespace vk = valhalla::loki;
namespace vj = valhalla::mjolnir;
namespace vo = valhalla::odin;

#include "mjolnir/directededgebuilder.h"
#include "mjolnir/graphtilebuilder.h"

namespace {

const std::string config_file = "test/test_trivial_path";

void write_config(const std::string& filename) {
  std::ofstream file;
  try {
    file.open(filename, std::ios_base::trunc);
    file << "{ \
      \"mjolnir\": { \
      \"concurrency\": 1, \
       \"tile_dir\": \"test/data/trivial_tiles\", \
        \"admin\": \"" VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite\", \
         \"timezone\": \"" VALHALLA_SOURCE_DIR "test/data/not_needed.sqlite\" \
      } \
    }";
  } catch (...) {}
  file.close();
}
void trivial_path_no_uturns(const std::string& config_file) {
  boost::property_tree::ptree conf;
  rapidjson::read_json(config_file, conf);

  // setup and purge
  vb::GraphReader graph_reader(conf.get_child("mjolnir"));
  for (const auto& level : vb::TileHierarchy::levels()) {
    auto level_dir = graph_reader.tile_dir() + "/" + std::to_string(level.first);
    if (boost::filesystem::exists(level_dir) && !boost::filesystem::is_empty(level_dir)) {
      boost::filesystem::remove_all(level_dir);
    }
  }

  // Set up the temporary (*.bin) files used during processing
  std::string ways_file = "test_ways_trivial.bin";
  std::string way_nodes_file = "test_way_nodes_trivial.bin";
  std::string nodes_file = "test_nodes_trivial.bin";
  std::string edges_file = "test_edges_trivial.bin";
  std::string access_file = "test_access_trivial.bin";
  std::string cr_from_file = "test_from_complex_restrictions_trivial.bin";
  std::string cr_to_file = "test_to_complex_restrictions_trivial.bin";
  std::string bss_nodes_file = "test_bss_nodes_file_trivial.bin";

  // Parse Utrecht OSM data
  auto osmdata =
      vj::PBFGraphParser::Parse(conf.get_child("mjolnir"),
                                {VALHALLA_SOURCE_DIR "test/data/utrecht_netherlands.osm.pbf"},
                                ways_file, way_nodes_file, access_file, cr_from_file, cr_to_file,
                                bss_nodes_file);

  // Build the graph using the OSMNodes and OSMWays from the parser
  vj::GraphBuilder::Build(conf, osmdata, ways_file, way_nodes_file, nodes_file, edges_file,
                          cr_from_file, cr_to_file);

  // Enhance the local level of the graph. This adds information to the local
  // level that is usable across all levels (density, administrative
  // information (and country based attribution), edge transition logic, etc.
  vj::GraphEnhancer::Enhance(conf, osmdata, access_file);

  // Validate the graph and add information that cannot be added until
  // full graph is formed.
  vj::GraphValidator::Validate(conf);

  // Locations
  std::vector<valhalla::baldr::Location> locations;
  baldr::Location origin(valhalla::midgard::PointLL(5.114587f, 52.095957f),
                         baldr::Location::StopType::BREAK);
  locations.push_back(origin);
  baldr::Location dest(valhalla::midgard::PointLL(5.114506f, 52.096141f),
                       baldr::Location::StopType::BREAK);
  locations.push_back(dest);

  Api api;
  auto& options = *api.mutable_options();
  create_costing_options(options);
  std::shared_ptr<vs::DynamicCost> mode_costing[4];
  std::shared_ptr<vs::DynamicCost> cost = vs::CreatePedestrianCost(Costing::pedestrian, options);
  auto mode = cost->travel_mode();
  mode_costing[static_cast<uint32_t>(mode)] = cost;

  const auto projections = vk::Search(locations, graph_reader, cost.get());
  std::vector<PathLocation> path_location;

  for (const auto& loc : locations) {
    ASSERT_NO_THROW(
        path_location.push_back(projections.at(loc));
        PathLocation::toPBF(path_location.back(), options.mutable_locations()->Add(), graph_reader);)
        << "fail_invalid_origin";
  }

  vt::AStarPathAlgorithm astar;
  auto path = astar
                  .GetBestPath(*options.mutable_locations(0), *options.mutable_locations(1),
                               graph_reader, mode_costing, mode)
                  .front();

  vt::AttributesController controller;
  auto& leg = *api.mutable_trip()->mutable_routes()->Add()->mutable_legs()->Add();
  vt::TripLegBuilder::Build(controller, graph_reader, mode_costing, path.begin(), path.end(),
                            *options.mutable_locations(0), *options.mutable_locations(1),
                            std::list<valhalla::Location>{}, leg);
  // really could of got the total of the elapsed_time.
  odin::DirectionsBuilder::Build(api);
  const auto& trip_directions = api.directions().routes(0).legs(0);

  EXPECT_EQ(trip_directions.summary().time(), 0);

  boost::filesystem::remove(ways_file);
  boost::filesystem::remove(way_nodes_file);
  boost::filesystem::remove(nodes_file);
  boost::filesystem::remove(edges_file);
  boost::filesystem::remove(access_file);
  boost::filesystem::remove(cr_from_file);
  boost::filesystem::remove(cr_to_file);
}

TEST(Astar, TestTrivialPathNoUturns) {
  write_config(config_file);
  trivial_path_no_uturns(config_file);
}

class AstarSlowTestEnv : public ::testing::Environment {
public:
  void SetUp() override {
  }
};

} // anonymous namespace

int main(int argc, char* argv[]) {
  testing::AddGlobalTestEnvironment(new AstarSlowTestEnv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
