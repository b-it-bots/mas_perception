/** @file linemod.h
 *  @brief Wrapper for LINEMOD object recognition
 *  @author    Alexander Hagg
 *  @memberof  b-it-bots RoboCup@Home team
 *  @version   1.0
 *  @date      7 March 2014
 *  @pre       First initialize the system.
 *  @copyright GNU Public License.
 */

#ifndef LINEMOD_H_
#define LINEMOD_H_

#include <string>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/recognition/linemod.h>
#include <pcl/recognition/sparse_quantized_multi_mod_template.h>
#include <pcl/recognition/color_gradient_modality.h>
#include <pcl/recognition/surface_normal_modality.h>

/** @brief Output wrapper class
 *
 *  Contains representation of output object
 * 
 *  @param name             The name (id) of the object
 *         similarity       Found similarity in the scene
 *         x, y            Origin of the template in the scene (in pixels)
 *         template_found  Best fit template
 *
 */
class ObjectRegion
{
 public:
    ObjectRegion(std::string name, float similarity = 0)
            : name(name),
              similarity(similarity),
              x(0),
              y(0),
              template_found(NULL),
              world_x(0.0),
              world_y(0.0),
              world_z(0.0)
    {
    }
    ObjectRegion(std::string name, float similarity, int x, int y, const pcl::SparseQuantizedMultiModTemplate *template_found = NULL)
            : name(name),
              similarity(similarity),
              x(x),
              y(y),
              template_found(template_found),
              world_x(0.0),
              world_y(0.0),
              world_z(0.0)
    {
    }
    ObjectRegion(std::string name, float similarity, int x, int y, const pcl::SparseQuantizedMultiModTemplate *template_found, float world_x, float world_y, float world_z)
            : name(name),
              similarity(similarity),
              x(x),
              y(y),
              template_found(template_found),
              world_x(world_x),
              world_y(world_y),
              world_z(world_z)
    {
    }

    std::string name;
    float similarity;
    int x;
    int y;
    float world_x;
    float world_y;
    float world_z;
    const pcl::SparseQuantizedMultiModTemplate *template_found;
};

/** @brief Linemod wrapper class
 *
 *  Contains LINEMOD detector and user methods
 * 
 *  @param database_folder_name 
 *         database_file_name    Database file prefix
 *         detection_threshold   Detection threshold
 *         num_modalities        Number of used modalities in the database, which should
 *                               be known to the user.
 * 
 */
class Linemod
{
 public:
    Linemod(std::string database_folder_name, std::string database_file_name, float detection_threshold, int num_modalities = 2);
    virtual ~Linemod();

    /** @brief Sets the mask that is used to mask off non-ROI points when learning
     * 
     *  @param roi*  minimum and maximum range for all dimensions
     */
    void setMask(float roi_min_x, float roi_max_x, float roi_min_y, float roi_max_y, float roi_min_z, float roi_max_z);

    /** @brief Returns all detected templates and their mapped names
     * 
     *  @param input Input point cloud
     * 
     *  @return A vector of all detected objects
     */
    std::vector<ObjectRegion> computeDetections(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &input);

    /** @brief Adds a template for a known object
     * 
     *  @param input Input point cloud
     *         object_id Object id string
     */
    void addTemplate(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &input, std::string object_id, bool new_object = false);

    /** @brief Adds a template for a new object
     * 
     *  @param input Input point cloud
     */
    void addTemplate(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &input);

    /** @brief Get object names from database
     * 
     *  @return Vector of name strings
     */
    std::vector<std::string> getObjectNames();


 private:
    /** @brief Masks non-ROI points
     * 
     *  @param input Input point cloud
     * 
     *  @return Mask vector
     */
    std::vector<bool> maskForegroundPoints(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &input);

    /** @brief Combines the input cloud and mask and saves the template
     * 
     *  @param input Input point cloud
     *         foreground_mask Mask vector
     */
    void trainTemplate(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &input, const std::vector<bool> &foreground_mask);

    /** @brief Matches the current templates to the input
     * 
     *  @param input Input point cloud
     * 
     *  @return Vector of detections
     */
    std::vector<pcl::LINEMODDetection> matchTemplates(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &input);

    double roi_min_x;
    double roi_max_x;
    double roi_min_y;
    double roi_max_y;
    double roi_min_z;
    double roi_max_z;
    std::string database_folder_name;
    std::string database_file_name;
    float detection_threshold;
    int num_modalities;
    pcl::LINEMOD object_detector;
    std::vector<std::string> map_object_id_name;
    std::vector<int> map_templates_objects;
    std::vector<std::vector<int> > map_objects_templates;
    std::vector<pcl::SparseQuantizedMultiModTemplate> templates;
};

#endif /* LINEMOD_H_ */