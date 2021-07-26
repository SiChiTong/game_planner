#include "lane/lane.h"

int main(int argc, char** argv)
{
    std::string lane_path = "/home/yutaka/game_theoretical_planner/src/simulator/lane_data/lane_circle.json";
    std::ifstream fs;
    fs.open(lane_path);
    if(!fs)
        return 1;

    std::string testJson;
    std::getline(fs, testJson);
    fs.close();

    Json::Reader reader;
    Json::Value root;
    reader.parse(testJson, root);

    Json::Value lane_net_json = root["features"];
    for(int i=0; i<lane_net_json.size(); ++i)
    {
        Json::Value lane_coordinates = lane_net_json[i]["geometry"]["coordinates"][0];
        int num_pts = static_cast<int>(lane_coordinates.size());
        for(int j=0; j<num_pts; ++j)
        {
            std::cout << "x: [" << i << "][" << j << "]: " << lane_coordinates[j][0] << std::endl;
            std::cout << "y: [" << i << "][" << j << "]: " << lane_coordinates[j][1] << std::endl;
        }
    }

    return 0;
}