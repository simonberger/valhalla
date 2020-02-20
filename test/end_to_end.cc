/******************************************************************************
 * End-to-end tests
 *
 * These include:
 *   1. Parsing an OSM map
 *   2. Generating tiles
 *   3. Calculating routes on tiles
 *   4. Verify the expected route
 ******************************************************************************/
#include "baldr/rapidjson_utils.h"
#include "loki/worker.h"
#include "mjolnir/util.h"
#include "odin/worker.h"
#include "test.h"
#include "thor/worker.h"
#include "util/mapgen.h"
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>

namespace {

namespace vtm = valhalla::test::mapgen;
namespace vj = valhalla::mjolnir;

boost::property_tree::ptree json_to_pt(const std::string& json) {
  std::stringstream ss;
  ss << json;
  boost::property_tree::ptree pt;
  rapidjson::read_json(ss, pt);
  return pt;
}

std::string build_simple_request(const vtm::nodemap& nodemap,
                                 const std::string& from,
                                 const std::string& to,
                                 const std::string& costing = "auto") {
  return "{\"locations\":[{\"lat\":" + std::to_string(nodemap.at(from).lat) +
         ",\"lon\":" + std::to_string(nodemap.at(from).lon) +
         "},{\"lat\":" + std::to_string(nodemap.at(to).lat) +
         ",\"lon\":" + std::to_string(nodemap.at(to).lon) + "}],\"costing\":\"" + costing + "\"}";
}

boost::property_tree::ptree build_config(const std::string& tiledir) {
  auto ptree = json_to_pt(R"(
    {"mjolnir":{"tile_dir":"", "concurrency": 1},
     "thor":{
       "logging" : {"long_request" : 100}
     },
     "meili":{
       "logging" : {"long_request" : 100},
       "grid" : {"cache_size" : 100, "size": 100 }
     },
     "loki":{
       "actions" : ["sources_to_targets"], 
       "logging" : {"long_request" : 100}, 
       "service_defaults" : {
         "minimum_reachability" : 50,
         "radius" : 0,
         "search_cutoff" : 35000,
         "node_snap_tolerance" : 5,
         "street_side_tolerance" : 5,
         "heading_tolerance" : 60
        }
     },
     "service_limits": {
      "auto": {"max_distance": 5000000.0, "max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "auto_shorter": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "bicycle": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50},
      "bus": {"max_distance": 5000000.0,"max_locations": 50,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "hov": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "taxi": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "isochrone": {"max_contours": 4,"max_distance": 25000.0,"max_locations": 1,"max_time": 120},
      "max_avoid_locations": 50,"max_radius": 200,"max_reachability": 100,"max_alternates":2,
      "multimodal": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 0.0,"max_matrix_locations": 0},
      "pedestrian": {"max_distance": 250000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50,"max_transit_walking_distance": 10000,"min_transit_walking_distance": 1},
      "skadi": {"max_shape": 750000,"min_resample": 10.0},
      "trace": {"max_distance": 200000.0,"max_gps_accuracy": 100.0,"max_search_radius": 100,"max_shape": 16000,"max_best_paths":4,"max_best_paths_shape":100},
      "transit": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50},
      "truck": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50}
    }
  })");
  ptree.put("mjolnir.tile_dir", tiledir);
  return ptree;
}

valhalla::Api build_and_route(const std::string& mapstring,
                              const double gridsize,
                              const vtm::props& ways,
                              const std::string& from,
                              const std::string& to,
                              const std::string& costing = "auto") {

  // Come up with a unique name for where to put the data for this test
  const ::testing::TestInfo* const test_info =
      ::testing::UnitTest::GetInstance()->current_test_info();
  const std::string testname = std::string(test_info->test_case_name()) + "_" + test_info->name();

  std::string testdir = "test/data/" + testname;
  const auto config = build_config(testdir);

  if (boost::filesystem::exists(testdir))
    boost::filesystem::remove_all(testdir);

  boost::filesystem::create_directories(testdir);

  auto nodemap = vtm::map_to_coordinates(mapstring, gridsize);
  auto pbf_filename = testdir + "/" + testname + ".pbf";
  vtm::build_pbf(nodemap, ways, {}, {}, pbf_filename);
  vj::build_tile_set(config, {pbf_filename}, vj::BuildStage::kInitialize, vj::BuildStage::kValidate,
                     false);

  valhalla::loki::loki_worker_t loki_worker(config);
  valhalla::thor::thor_worker_t thor_worker(config);
  valhalla::odin::odin_worker_t odin_worker(config);
  valhalla::baldr::GraphReader reader(config.get_child("mjolnir"));
  valhalla::Api request;

  auto request_json = build_simple_request(nodemap, from, to, costing);

  valhalla::ParseApi(request_json.c_str(), valhalla::Options::route, request);
  loki_worker.route(request);
  std::pair<std::list<valhalla::TripLeg>, std::list<valhalla::DirectionsLeg>> results;
  thor_worker.route(request);
  odin_worker.narrate(request);
  return request;
}

/************************************************************************************/
TEST(EndToEnd, TurnRight) {

  const std::string map = R"(
    A----B----C
         |
         D)";

  const vtm::props ways = {{"ABC", {{"highway", "primary"}}}, {"BD", {{"highway", "primary"}}}};

  const std::string from = "A";
  const std::string to = "D";
  const std::string costing = "auto";
  const double gridsize = 100;
  auto result = build_and_route(map, gridsize, ways, from, to, costing);

  // Only expect one possible route
  EXPECT_EQ(result.directions().routes_size(), 1);
  EXPECT_EQ(result.directions().routes(0).legs_size(), 1);

  auto& leg = result.directions().routes(0).legs(0);
  EXPECT_EQ(leg.maneuver(0).type(), valhalla::DirectionsLeg_Maneuver_Type_kStart);
  EXPECT_EQ(leg.maneuver(1).type(), valhalla::DirectionsLeg_Maneuver_Type_kRight);
  EXPECT_EQ(leg.maneuver(2).type(), valhalla::DirectionsLeg_Maneuver_Type_kDestination);

  EXPECT_EQ(leg.maneuver(0).street_name(0).value(), std::string("ABC"));
  EXPECT_EQ(leg.maneuver(1).street_name(0).value(), std::string("BD"));
}
/************************************************************************************/
TEST(EndToEnd, TurnLeft) {

  const std::string map = R"(
    A----B----C
         |
         D)";

  const vtm::props ways = {{"ABC", {{"highway", "primary"}}}, {"BD", {{"highway", "primary"}}}};

  const std::string from = "D";
  const std::string to = "A";
  const std::string costing = "auto";
  const double gridsize = 100;
  auto result = build_and_route(map, gridsize, ways, from, to, costing);

  // Only expect one possible route
  EXPECT_EQ(result.directions().routes_size(), 1);
  EXPECT_EQ(result.directions().routes(0).legs_size(), 1);

  auto& leg = result.directions().routes(0).legs(0);
  EXPECT_EQ(leg.maneuver(0).type(), valhalla::DirectionsLeg_Maneuver_Type_kStart);
  EXPECT_EQ(leg.maneuver(1).type(), valhalla::DirectionsLeg_Maneuver_Type_kLeft);
  EXPECT_EQ(leg.maneuver(2).type(), valhalla::DirectionsLeg_Maneuver_Type_kDestination);

  EXPECT_EQ(leg.maneuver(0).street_name(0).value(), std::string("BD"));
  EXPECT_EQ(leg.maneuver(1).street_name(0).value(), std::string("ABC"));
}
/************************************************************************************/

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
