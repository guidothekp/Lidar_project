/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // Add inliers
    float scatter = 0.6;
    for(int i = -5; i < 5; i++)
    {
        double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
        double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
        pcl::PointXYZ point;
        point.x = i+scatter*rx;
        point.y = i+scatter*ry;
        point.z = 0;

        cloud->points.push_back(point);
    }
    // Add outliers
    int numOutliers = 10;
    while(numOutliers--)
    {
        double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
        double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
        pcl::PointXYZ point;
        point.x = 5*rx;
        point.y = 5*ry;
        point.z = 0;

        cloud->points.push_back(point);

    }
    cloud->width = cloud->points.size();
    cloud->height = 1;

    return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
    viewer->addCoordinateSystem (1.0);
    return viewer;
}

void fillARandomNumber(int & total, std::unordered_set<int> & current) {
    int number = rand() % total;
    while (current.count(number) > 0) {
        number = rand() % total;
    }
    current.insert(number);
}

std::vector<float> toVector(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int & first, int & second) {
    std::vector<float> v;
    pcl::PointXYZ firstPoint = cloud->points.at(first), secondPoint = cloud->points.at(second);
    v.push_back(secondPoint.x - firstPoint.x);
    v.push_back(secondPoint.y - firstPoint.y);
    v.push_back(secondPoint.z - firstPoint.z);
    return v;
}

void findPlane(std::vector<float> & v1, std::vector<float> & v2, pcl::PointXYZ point, std::vector<float> & coefficients) {
    float d = 0;
    for (int i = 0; i < 3; i ++) {
        int first = (i + 1) % 3, second = (i + 2) % 3;
        coefficients.push_back(v1[first] * v2[second] - v1[second] * v2[first]);
    }
    std::vector<float> pointValues = {point.x, point.y, point.z};
    for (int i = 0; i < 3; i ++) {
        d += coefficients[i] * pointValues[i];
    }
    coefficients.push_back(-d);
}

std::vector<float> fitAPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    int total = cloud->points.size();
    std::unordered_set<int> pointIndices; 
    for (int i = 0; i < 3; i ++)
        fillARandomNumber(total, pointIndices);
    int indices[3];
    auto itr = pointIndices.begin();
    int counter = 0;
    for(; itr != pointIndices.end();  itr++, counter++) indices[counter] = *itr;
    std::vector<float> v1 = toVector(cloud, indices[0], indices[1]);
    std::vector<float> v2 = toVector(cloud, indices[1], indices[2]);
    std::vector<float> coefficients;
    findPlane(v1, v2, cloud->points.at(2), coefficients);
    return coefficients;
}

std::vector<int> fitALine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    int total = cloud->points.size();
    int first = rand() % total;
    int second = rand() % total;
    while (second == first) {
        second = rand() % total;
    }
    pcl::PointXYZ fp = cloud->points.at(first), sp = cloud->points.at(second);
    int y1  = fp.y, y2 = sp.y, x1 = fp.x, x2 = sp.y;
    std::vector<int> v = {y1 - y2, x2 - x1, x1 * y2 - x2 * y1};
    return v;
}

void findInliers(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float threshold,
        std::vector<int> & coefficients, std::vector<int> & inliers) {
    float A = coefficients.at(0), B = coefficients.at(1), C = coefficients.at(2), length = sqrt(A * A + B * B);
    for (int i = 0; i < cloud->points.size(); i ++) {
        pcl::PointXYZ point = cloud->points.at(i);
        float x = point.x, y = point.y;
        float distance = fabs(A * x + B* y + C)/length;
        if (distance < threshold) {
            inliers.push_back(i);
        }
    }	
}

template<class T> void print(const std::unordered_set<T> & v) {
    for (auto itr = v.begin(); itr != v.end(); itr++) {
        std::cout << *itr;
        if (itr != v.end()) {
            std::cout << ", ";
        }
    }
    std::cout << std::endl;
}

pcl::PointXYZ findCrossProduct(const pcl::PointXYZ & u, const pcl::PointXYZ & v) {
    return pcl::PointXYZ {
        u.y * v.z - u.z * v.y,
            u.z * v.x - u.x * v.z,
            u.x * v.y - u.y * v.x
    };
}

float findDotProduct(const pcl::PointXYZ & a, const pcl::PointXYZ & b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

pcl::PointXYZ findVector(const pcl::PointXYZ & start, const pcl::PointXYZ & end) {
    return pcl::PointXYZ {
        end.x - start.x,
            end.y - start.y,
            end.z - start.z
    };
}

/**
 * Given three points, find a plane through them.
 */
std::vector<float> find3Plane(std::unordered_set<int> & indices, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    auto iterator = indices.begin();
    const auto & point1 {cloud->points[*iterator ++]};
    const auto & point2 {cloud->points[*iterator ++]};
    const auto & point3 {cloud->points[*iterator ++]};

    const auto & v1  = findVector(point1, point2);
    const auto & v2  = findVector(point2, point3);


    const auto & crossProduct = findCrossProduct(v1, v2);
    const auto & A {crossProduct.x};
    const auto & B {crossProduct.y};
    const auto & C {crossProduct.z};
    const auto & D {-1 * findDotProduct(crossProduct, point1)};

#ifdef VERBOSE
    for (const auto p: indices) {
        std::cout << "point selected: " << cloud->points[p] << std::endl;
    }
    std::cout << "point1: " << point1 << std::endl;
    std::cout << "point2: " << point2 << std::endl;
    std::cout << "point3: " << point3 << std::endl;
    std::cout << "v1: " << v1 << std::endl;
    std::cout << "v2: " << v2 << std::endl;
    std::cout << "crossproduct: " << crossProduct << std::endl;
#endif
    return std::vector {A, B, C, D};
}

float findDistance(const std::vector<float> & coefficients, const pcl::PointXYZ & p) {
    const auto & A {coefficients[0]}, B{coefficients[1]}, C{coefficients[2]}, D{coefficients[3]};
    return fabs(A * p.x + B * p.y + C * p.z + D)/sqrt(A*A + B*B + C*C);
}


std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        int maxIterations, float threshold) {
    std::unordered_set<int> indices;
    for (int i = 0; i < maxIterations; i ++) {
        std::unordered_set<int> pointIndices;
        while (pointIndices.size() < 3 ) {
            pointIndices.insert(rand() % cloud->points.size());
        }
        const auto & plane = find3Plane(pointIndices, cloud);
#ifdef VERBOSE
        for (const auto & c : plane) {
            std::cout << c << " ";
        }
        std::cout << "\n";
#endif
        for (int j = 0; j < cloud->points.size(); j ++) {
            const auto & d = findDistance(plane, cloud->points[j]);
            if (d < threshold) {
                pointIndices.insert(j);
            }
        }
        if (pointIndices.size() > indices.size()) {
            indices = pointIndices;
        }
    }
    return indices;
}

std::unordered_set<int> RansacPlane2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        int maxIterations, float threshold) {
    std::unordered_set<int> inliersResult, pointIndices, currentInliers;
    std::vector<float> coefficients;
    int total = cloud->points.size();
    srand(time(NULL));
    for (int i = 0; i < maxIterations; i ++) {
        coefficients.clear();
        currentInliers.clear();		
        pointIndices.clear();
        //choose 3 points
        while (pointIndices.size() < 3) {
            pointIndices.insert(rand() % cloud->points.size());
        }
        /*for (int j = 0; j < 3; j ++) {
          int pj = rand() % total;
          while (pointIndices.count(pj) > 0) {
          pj = rand() % total;
          }
          pointIndices.insert(pj);
          }*/
        std::vector<int> pis;
        for (auto itr = pointIndices.begin(); itr != pointIndices.end(); pis.push_back(*itr ),itr ++);
        std::vector<float> v[2]; //vectors
        for (int j = 0; j < 2; j ++) {
            auto fp = cloud->points[pis[j]], sp = cloud->points[pis[j + 1]];
            v[j].push_back(sp.x - fp.x);
            v[j].push_back(sp.y - fp.y);
            v[j].push_back(sp.z - fp.z);
        }
        //cross product of vectors
        for (int j = 0; j < 3; j ++) {
            int fi = (j + 1) % 3, si = (j + 2) %3;
            float f = v[0][fi] * v[1][si] - v[0][si] * v[1][fi];
            coefficients.push_back(f); //v[0][fi] * v[1][si] - v[0][si] * v[1][fi];
        }
        auto p1 = cloud->points[pis[0]];		
        coefficients.push_back(-(p1.x * coefficients[0] + p1.y * coefficients[1] + p1.z * coefficients[2]));
        for (int j = 0; j < cloud->points.size(); j ++) {
            auto p = cloud->points[j];
            float coords[] = {p.x, p.y, p.z, 1};
            float distance = 0.0, length = 0;
            for (int k = 0; k < coefficients.size(); k ++) {
                distance += coefficients[k] * coords[k];
            }
            for (int k = 0; k < 3; k ++) {
                length += coords[k] * coords[k];
            }
            distance = std::fabs(distance) / sqrt(length);
            if (distance < threshold) {
                currentInliers.insert(j);
            }
        }
        if (currentInliers.size() > inliersResult.size()) {
            inliersResult = currentInliers;
            // print<int>(inliersResult);
        }
    }	
    std::cout << inliersResult.size() << std::endl;
    // print<int>(inliersResult);
    return inliersResult;
}

// template<typename T, template<typename> class Container> void print(Container<T, std::allocator<T>> v) {}


template<class T> void print(const std::vector<T> & v) {
    using std::cout;
    using std::endl;
    for (int i = 0; i < v.size(); i ++) {
        cout << v.at(i) << ", ";
    }
    cout << endl;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::cout << "Ransac called." << std::endl;
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    // TODO: Fill in this function
    std::vector<int> currentInliers, inliers; //store the qualified inliers here.
    // For max iterations 
    for (int i = 0; i < maxIterations; i ++) {
        inliers.clear();
        std::vector<int> coefficients = fitALine(cloud);
        findInliers(cloud, distanceTol, coefficients, inliers);
        if (inliers.size() > currentInliers.size()) {
            currentInliers.clear();
            currentInliers = inliers;
            cout << "--------------\nfound new inliers at iteration = " << i << ", size = " << currentInliers.size() << endl;
            print<int>(currentInliers);
        }
    }
    // for (int i = 0; i < inliers.size(); i ++) {
    // std::cout << inliers.at(i) << std::endl;
    // }
    std::cout << "---------" << std::endl;
    std::copy(currentInliers.begin(), currentInliers.end(), std::inserter(inliersResult, inliersResult.end()));
    // Randomly sample subset and fit line

    // Measure distance between every point and fitted line
    // If distance is smaller than threshold count it as inlier

    // Return indicies of inliers from fitted line with most inliers
    for (int i : std::as_const(inliersResult)) {
        std::cout << i << std::endl;	
    }
    return inliersResult;
}



int main ()
{

    // Create viewer
    pcl::visualization::PCLVisualizer::Ptr viewer = initScene();
    // 
    // Create data
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();


    // TODO: Change the max iteration and distance tolerance arguments for Ransac function
    //std::unordered_set<int> inliers = Ransac(cloud, 0, 0);
    // std::unordered_set<int> inliers = Ransac(cloud, 150, 0.5);
    std::unordered_set<int> inliers = RansacPlane(cloud, 50, 0.5);

    pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

    for(int index = 0; index < cloud->points.size(); index++)
    {
        pcl::PointXYZ point = cloud->points[index];
        if(inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    std::cout << "inliers count: " << inliers.size() << std::endl;
    // Render 2D point cloud with inliers and outliers
    if(inliers.size())
    {
        renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
        renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
    }
    else
    {
        renderPointCloud(viewer,cloud,"data");
    }

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    }

}
