/** @file linemod.cpp
 *  @brief Wrapper for LINEMOD object recognition
 *  @author    Alexander Hagg
 *  @memberof  b-it-bots RoboCup@Home team
 *  @version   1.0 
 *  @date      7 March 2014
 *  @pre       First initialize the system.
 *  @copyright GNU Public License.
 */

 #include <mcr_object_recognition_linemod/linemod.h>

 Linemod::Linemod(std::string database_folder_name, std::string database_file_name, float detection_threshold, 
                  int num_modalities)
        : database_folder_name(database_folder_name),
          database_file_name(database_file_name),
          num_modalities(num_modalities),
          detection_threshold(detection_threshold)
{
    object_detector.setDetectionThreshold (detection_threshold);

    // Load templates from database files and store mappings
    boost::filesystem::recursive_directory_iterator dir(database_folder_name);
    boost::filesystem::recursive_directory_iterator end;
    int current_object = 0;
    pcl::LINEMOD file_loader;

    while (dir != end)
    { 
        std::vector<int> map_object_template;
        file_loader.loadTemplates((database_folder_name + dir->path().filename().string()).c_str());

        for (unsigned int templ = 0; templ < file_loader.getNumOfTemplates(); templ++)
        {
            templates.push_back(file_loader.getTemplate(templ));
        }

        map_object_id_name.push_back(dir->path().filename().string());

        for (unsigned int i = 0; i < file_loader.getNumOfTemplates(); i++)
        {
            map_templates_objects.push_back(current_object);
            map_object_template.push_back(i);
        }

        map_objects_templates.push_back(map_object_template);
        current_object++;
        dir++;
    }

    // Load all templates into object detector
    for (unsigned int templ = 0; templ < templates.size(); templ++)
    {
        object_detector.addTemplate(templates.at(templ));
    }
}

Linemod::~Linemod()
{
}

void Linemod::setMask(float roi_min_x, float roi_max_x, float roi_min_y, float roi_max_y, float roi_min_z, 
                      float roi_max_z)
{
    this->roi_min_x = roi_min_x;
    this->roi_max_x = roi_max_x;
    this->roi_min_y = roi_min_y;
    this->roi_max_y = roi_max_y;
    this->roi_min_z = roi_min_z;
    this->roi_max_z = roi_max_z;
}

std::vector<ObjectRegion> Linemod::computeDetections(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &input)
{
    std::vector<ObjectRegion> detected_objects;
    std::vector<pcl::LINEMODDetection> detections = matchTemplates(input);
    for (size_t detection = 0; detection < detections.size(); detection++)
    {
        const pcl::LINEMODDetection &d = detections.at(detection);
        int detected_object_id = map_templates_objects.at(d.template_id);
        ObjectRegion new_object(map_object_id_name.at(detected_object_id), d.score, d.x, d.y, 
                                &object_detector.getTemplate(d.template_id));
        detected_objects.push_back(new_object); 
    } 
    return detected_objects;
}

void Linemod::addTemplate(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &input, std::string object_id, bool new_object)
{
    std::vector<bool> foreground_mask = maskForegroundPoints(input);
    trainTemplate(input, foreground_mask);
    
    pcl::LINEMOD db_writer;
    std::string template_file = database_folder_name + object_id;
    if (!new_object)
    {
        db_writer.loadTemplates(template_file.c_str());
    }
    db_writer.addTemplate(object_detector.getTemplate(object_detector.getNumOfTemplates() - 1));
    db_writer.saveTemplates(template_file.c_str());

    if (new_object) 
    {
        map_templates_objects.push_back(map_object_id_name.size());  
        std::vector<int> map_object_template;
        map_object_template.push_back(object_detector.getNumOfTemplates()-1);
        map_objects_templates.push_back(map_object_template);
        map_object_id_name.push_back(object_id);
    }

    int object_id_int = -1;
    for (size_t j = 0; j < map_object_id_name.size(); j++)
    {
        if (map_object_id_name.at(j).compare(object_id) == 0)
        {
            object_id_int = j;
        }
    }

    // As this method is called for known objects, the object_id needs to be in the id->name mapping
    if (object_id_int == -1)
    {
        return;
    }

    map_templates_objects.push_back(object_id_int);  
    map_objects_templates.at(object_id_int).push_back(object_detector.getNumOfTemplates()-1);
}

void Linemod::addTemplate(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &input)
{
    std::stringstream ss;
    ss << map_objects_templates.size();
    std::string new_object_id = database_file_name + ss.str();
    std::string template_file = database_folder_name + new_object_id;

    std::vector<bool> foreground_mask = maskForegroundPoints (input);
    trainTemplate(input, foreground_mask);

    pcl::LINEMOD db_writer;
    db_writer.addTemplate(object_detector.getTemplate(object_detector.getNumOfTemplates()-1));
    db_writer.saveTemplates(template_file.c_str());

    map_templates_objects.push_back(map_object_id_name.size());  
    std::vector<int> map_object_template;
    map_object_template.push_back(object_detector.getNumOfTemplates()-1);
    map_objects_templates.push_back(map_object_template);
    map_object_id_name.push_back(new_object_id);
}

std::vector<pcl::LINEMODDetection> Linemod::matchTemplates(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &input)
{
    pcl::ColorGradientModality<pcl::PointXYZRGBA> color_grad_mod;
    color_grad_mod.setInputCloud(input);
    color_grad_mod.processInputData();

    pcl::SurfaceNormalModality<pcl::PointXYZRGBA> surface_norm_mod;
    surface_norm_mod.setInputCloud(input);
    surface_norm_mod.processInputData();

    std::vector<pcl::QuantizableModality *> modalities(2);
    modalities[0] = &color_grad_mod;
    modalities[1] = &surface_norm_mod;

    std::vector<pcl::LINEMODDetection> detections;
    object_detector.matchTemplates(modalities, detections);

    return detections;
}

void Linemod::trainTemplate(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &input, const std::vector<bool> &foreground_mask)
{
    pcl::ColorGradientModality<pcl::PointXYZRGBA> color_grad_mod;
    color_grad_mod.setInputCloud(input);
    color_grad_mod.processInputData();

    pcl::SurfaceNormalModality<pcl::PointXYZRGBA> surface_norm_mod;
    surface_norm_mod.setInputCloud(input);
    surface_norm_mod.processInputData();

    std::vector<pcl::QuantizableModality *> modalities(2);
    modalities[0] = &color_grad_mod;
    modalities[1] = &surface_norm_mod;

    size_t min_x(input->width);
    size_t min_y(input->height);
    size_t max_x(0);
    size_t max_y(0);
    pcl::MaskMap mask_map (input->width, input->height);

    for (size_t j = 0; j < input->height; ++j)
    {
        for (size_t i = 0; i < input->width; ++i)
        {
            mask_map (i,j) = foreground_mask[j * input->width + i];
            if (foreground_mask[j * input->width + i])
            {
                min_x = std::min(min_x, i);
                max_x = std::max(max_x, i);
                min_y = std::min(min_y, j);
                max_y = std::max(max_y, j);
            }
        }
    }

    std::vector<pcl::MaskMap *> masks(2);
    masks[0] = &mask_map;
    masks[1] = &mask_map;

    pcl::RegionXY region;
    region.x = static_cast<int>(min_x);
    region.y = static_cast<int>(min_y);
    region.width = static_cast<int>(max_x - min_x + 1);
    region.height = static_cast<int>(max_y - min_y + 1);

    object_detector.createAndAddTemplate(modalities, masks, region);
}

std::vector<bool> Linemod::maskForegroundPoints(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &input)
{
    std::vector<bool> foreground_mask(input->size(), false);
    pcl::IndicesPtr indices(new std::vector<int>);

    for (size_t i = 0; i < input->size (); ++i)
    {
        const float x = input->points[i].x;
        const float y = input->points[i].y;
        const float z = input->points[i].z;
        if (roi_min_x < x && x < roi_max_x && roi_min_y < y && y < roi_max_y && roi_min_z < z && z < roi_max_z)
        {
            foreground_mask[i] = true;
            indices->push_back(static_cast<int>(i));
        }
    }

    return foreground_mask;
}

std::vector<std::string> Linemod::getObjectNames()
{
    return map_object_id_name;
}