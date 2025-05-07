#include "System.h"

#include <iostream>
#include <string>

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    std::string path_to_vocabulary = argv[1];
    std::string path_to_settings = argv[2];
    
    ORB_SLAM3::System SLAM(path_to_vocabulary, path_to_settings, ORB_SLAM3::System::RGBL, true);

    auto allMaps = SLAM.GetAtlas()->GetAllMaps();
    if (!allMaps.empty()) {
        SLAM.GetAtlas()->ChangeMap(allMaps[0]);
    } else {
        std::cerr << "No maps found in atlas!" << std::endl;
        return 1;
    }

    std::cout << "Map loaded, exporting..." << std::endl;

    SLAM.ExportMapPointsToPLY("map_points.ply");

    std::cout << "Map exported, done." << std::endl;

    SLAM.Shutdown();

    return 0;
}
