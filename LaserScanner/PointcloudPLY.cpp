#include "PointcloudPLY.h"

bool Pointcloud::write_ply(const std::string& filename, Pointcloud& pointcloud, unsigned flags)
{
    if (!pointcloud.points.data
        || (pointcloud.colors.data && pointcloud.colors.rows != pointcloud.points.rows && pointcloud.colors.cols != pointcloud.points.cols)
        || (pointcloud.normals.data && pointcloud.normals.rows != pointcloud.points.rows && pointcloud.normals.cols != pointcloud.points.cols))
    {
        return false;
    }

    bool binary = true;// true; // (flags&PlyBinary);
    bool colors = false; // (flags&PlyColors) && pointcloud.colors.data;
    bool normals = true;// (flags & PlyNormals) && pointcloud.normals.data;
    std::vector<int> points_index;
    points_index.reserve(pointcloud.points.total());

    const cv::Vec3f* points_data = pointcloud.points.ptr<cv::Vec3f>(0);
    const cv::Vec3b* colors_data = (colors ? pointcloud.colors.ptr<cv::Vec3b>(0) : NULL);
    const cv::Vec3f * normals_data = (normals ? pointcloud.normals.ptr<cv::Vec3f>(0) : NULL);

    int total = static_cast<int>(pointcloud.points.total());
    for (int i = 0; i < total; i++)
    {
        points_index.push_back(i);
    }

    std::ofstream outfile;
    std::ios::openmode mode = std::ios::out | std::ios::trunc | (binary ? std::ios::binary : static_cast<std::ios::openmode>(0));
    outfile.open(filename.c_str(), mode);
    if (!outfile.is_open())
    {
        return false;
    }

    const char* format_header = (binary ? "binary_little_endian 1.0" : "ascii 1.0");
    outfile << "ply" << std::endl
        << "format " << format_header << std::endl
        << "comment scan3d-capture generated" << std::endl
        << "element vertex " << points_index.size() << std::endl
        << "property float x" << std::endl
        << "property float y" << std::endl
        << "property float z" << std::endl;
    if (normals)
    {
        outfile << "property float nx" << std::endl
            << "property float ny" << std::endl
            << "property float nz" << std::endl;
    }
    if (colors)
    {
        outfile << "property uchar red" << std::endl
            << "property uchar green" << std::endl
            << "property uchar blue" << std::endl
            << "property uchar alpha" << std::endl;
    }
    outfile << "element face 0" << std::endl
        << "property list uchar int vertex_indices" << std::endl
        << "end_header" << std::endl;

    for (std::vector<int>::const_iterator iter = points_index.begin(); iter != points_index.end(); iter++)
    {
        cv::Vec3f const& p = points_data[*iter];
        if (binary)
        {
            outfile.write(reinterpret_cast<const char*>(&(p[0])), sizeof(float));
            outfile.write(reinterpret_cast<const char*>(&(p[1])), sizeof(float));
            outfile.write(reinterpret_cast<const char*>(&(p[2])), sizeof(float));
            if (normals)
            {
                cv::Vec3f const& n = -normals_data[*iter];
                outfile.write(reinterpret_cast<const char *>(&(n[0])), sizeof(float));
                outfile.write(reinterpret_cast<const char *>(&(n[1])), sizeof(float));
                outfile.write(reinterpret_cast<const char *>(&(n[2])), sizeof(float));
            }
            if (colors)
            {
                cv::Vec3b const& c = colors_data[*iter];
                const unsigned char a = 255U;
                outfile.write(reinterpret_cast<const char*>(&(c[2])), sizeof(unsigned char));
                outfile.write(reinterpret_cast<const char*>(&(c[1])), sizeof(unsigned char));
                outfile.write(reinterpret_cast<const char*>(&(c[0])), sizeof(unsigned char));
                outfile.write(reinterpret_cast<const char*>(&a), sizeof(unsigned char));
            }
        }
        else
        {
            outfile << p[0] << " " << p[1] << " " << p[2];
            if (normals)
            {
                cv::Vec3f const& n = normals_data[*iter];
                outfile << " " << n[0] << " " << n[1] << " " << n[2];
            }
            if (colors)
            {
                cv::Vec3b const& c = colors_data[*iter];
                outfile << " " << static_cast<int>(c[2]) << " " << static_cast<int>(c[1]) << " " << static_cast<int>(c[0]) << " 255";
            }
            outfile << std::endl;
        }
    }

    outfile.close();
    std::cerr << "[write_ply] Saved " << points_index.size() << " points (" << filename << ")" << std::endl;
    return true;
}
void Pointcloud::clear(void)
{
    points = cv::Mat();
    colors = cv::Mat();
    normals = cv::Mat();
}
void Pointcloud::init_points(int rows, int cols)
{
    points = cv::Mat(rows, cols, CV_32FC3);
    size_t total = points.total() * points.channels();
    float* data = points.ptr<float>(0);
    for (size_t i = 0; i < total; i++)
    {
        data[i] = std::numeric_limits<float>::quiet_NaN();
    }
}
void Pointcloud::init_color(int rows, int cols)
{
    colors = cv::Mat::zeros(rows, cols, CV_8UC3);
    memset(colors.data, 255, colors.total() * colors.channels()); //white
}
void Pointcloud::init_normals(int rows, int cols)
{
    normals = cv::Mat(rows, cols, CV_32FC3);
    size_t total = normals.total() * normals.channels();;
    float* data = normals.ptr<float>(0);
    for (size_t i = 0; i < total; i++)
    {
        data[i] = std::numeric_limits<float>::quiet_NaN();
    }
}
void Pointcloud::compute_normals(Pointcloud& pointcloud)
{
    if (!pointcloud.points.data)
    {
        return;
    }

    pointcloud.init_normals(pointcloud.points.rows, pointcloud.points.cols);

    for (int h = 1; h + 1 < pointcloud.points.rows; h++)
    {
        const cv::Vec3f* points_row0 = pointcloud.points.ptr<cv::Vec3f>(h - 1);
        const cv::Vec3f* points_row1 = pointcloud.points.ptr<cv::Vec3f>(h);
        const cv::Vec3f* points_row2 = pointcloud.points.ptr<cv::Vec3f>(h + 1);

        cv::Vec3f* normals_row = pointcloud.normals.ptr<cv::Vec3f>(h);

        for (int w = 1; w + 1 < pointcloud.points.cols; w++)
        {
            cv::Vec3f const& w1 = points_row1[w - 1];
            cv::Vec3f const& w2 = points_row1[w + 1];

            cv::Vec3f const& h1 = points_row0[w];
            cv::Vec3f const& h2 = points_row2[w];

            if (INVALID1(w1[0]) || INVALID1(w2[0]) || INVALID1(h1[0]) || INVALID1(h2[0]))
            {
                continue;
            }

            cv::Point3d n1(w2[0] - w1[0], w2[1] - w1[1], w2[2] - w1[2]);
            cv::Point3d n2(h2[0] - h1[0], h2[1] - h1[1], h2[2] - h1[2]);

            cv::Point3d normal = cv::Point3d(-n2.z * n1.y + n2.y * n1.z, n2.z * n1.x - n2.x * n1.z, -n2.y * n1.x + n2.x * n1.y);
            double norm = std::sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
            if (norm > 0.0)
            {
                cv::Vec3f& cloud_normal = normals_row[w];
                cloud_normal[0] = -normal.x / norm;
                cloud_normal[1] = -normal.y / norm;
                cloud_normal[2] = -normal.z / norm;
            }
        }
    }
}

