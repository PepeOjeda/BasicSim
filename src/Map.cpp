#include <basic_sim/Map.hpp>
#include <basic_sim/Logging.hpp>
#include <basic_sim/ConvertYAML.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <filesystem>

bool Map::readFile(const std::string& mapYAMLfilepath)
{
    try
    {
        YAML::Node yaml = YAML::LoadFile(mapYAMLfilepath);

        resolution = yaml["resolution"].as<double>();
        origin = yaml["origin"].as<tf2::Vector3>();

        double free_thresh =
            yaml["free_thresh"].as<double>(); // Unused, this is for trinary maps. We only consider cells free or occupied, not "unknown"
        double occupied_thresh = yaml["occupied_thresh"].as<double>();

#if 0
    //TODO maybe add support for map modes and negation?
    auto map_mode_node = doc["mode"];
    if (!map_mode_node.IsDefined()) {
        load_parameters.mode = MapMode::Trinary;
    } else {
        load_parameters.mode = map_mode_from_string(map_mode_node.as<std::string>());
    }

    try {
        load_parameters.negate = yaml_get_value<int>(doc, "negate");
    } catch (YAML::Exception &) {
        load_parameters.negate = yaml_get_value<bool>(doc, "negate");
    }
#endif

        // if the image path is relative interpret it as relative to the YAML, not the working directory
        std::filesystem::path imagePath(yaml["image"].as<std::string>());
        if(imagePath.is_relative())
            imagePath = std::filesystem::path(mapYAMLfilepath).parent_path()/imagePath;

        cv::Mat mapImage = cv::imread(imagePath, cv::IMREAD_GRAYSCALE);
        width = mapImage.size().width;
        height = mapImage.size().height;
        occupancyGrid.resize(width * height);
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                if (mapImage.at<uint8_t>(y, x) > 255 * (1-occupied_thresh))
                    occupancyGrid[width * (height - y - 1) + x] = CellState::Free;
                else
                    occupancyGrid[width * (height - y - 1) + x] = CellState::Occupied;
            }
        }
    }
    catch(std::exception& e)
    {
        BS_ERROR("%s", e.what());
        return false;
    }

    return true;
}



CellState Map::at(int i, int j) const
{
    int index = i + j * width;
    if(index <0 || index > occupancyGrid.size())
        return CellState::Occupied;
        
    return occupancyGrid[index];
}

CellState Map::at(const tf2::Vector3& point) const
{
    int i = (point.x() - origin.x()) / resolution;
    int j = (point.y() - origin.y()) / resolution;
    return at(i, j);
}

nav_msgs::msg::OccupancyGrid Map::asOccupancyGrid() const
{
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.frame_id="map";
    msg.info.height = height;
    msg.info.width = width;
    msg.info.resolution = resolution;
    msg.info.origin.position.x = origin.x();
    msg.info.origin.position.y = origin.y();
    msg.info.origin.position.z = origin.z();

    std::transform(occupancyGrid.cbegin(), occupancyGrid.cend(), std::back_inserter(msg.data),
               [](CellState a) { return static_cast<int8_t>(a); });
    return msg;
}


DDA::_2D::Map<CellState> Map::asDDAMap() const
{
    DDA::_2D::Map<CellState> DDAMap;
    DDAMap.origin = DDA::Vector2(origin.x(), origin.y());
    DDAMap.resolution = resolution;
    DDAMap.cells.resize(width, std::vector<CellState>(height));
    
    #pragma omp parallel for collapse(2)
    for(int i = 0; i<width; i++)
        for(int j = 0; j<height; j++)
            DDAMap.cells[i][j] = at(i,j);
    return DDAMap;
}
