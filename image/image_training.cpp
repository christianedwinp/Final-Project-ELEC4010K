#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

using namespace cv;
using namespace std;

// holds images and labels
vector<Mat> images;
vector<int> labels;

int main () {
    std::string directories;
    // images for first person
    for (int i = 0; i < 42; i++) {
        std::stringstream training_directories;
        if (i < 10) {
            training_directories << "person1/0" << i << ".jpg";
        } else {
            training_directories << "person1/" << i << ".jpg";
        }
        directories = training_directories.str();
        images.push_back(imread(directories, CV_LOAD_IMAGE_GRAYSCALE)); labels.push_back(0);    
    }
    
    // images for second person
    for (int i = 0; i < 49; i++) {
        std::stringstream training_directories;
        if (i < 10) {
            training_directories << "person2/0" << i << ".jpg";
        } else {
            training_directories << "person2/" << i << ".jpg";
        }
        directories = training_directories.str();
        images.push_back(imread(directories, CV_LOAD_IMAGE_GRAYSCALE)); labels.push_back(1);    
    }

    // images for third person
    for (int i = 0; i < 110; i++) {
        std::stringstream training_directories;
        if (i < 10) {
            training_directories << "person3/0" << i << ".jpg";
        } else {
            training_directories << "person3/" << i << ".jpg";
        }
        directories = training_directories.str();
        images.push_back(imread(directories, CV_LOAD_IMAGE_GRAYSCALE)); labels.push_back(2);    
    }

    // images for fourth person
    for (int i = 0; i < 26; i++) {
        std::stringstream training_directories;
        if (i < 10) {
            training_directories << "person4/0" << i << ".jpg";
        } else {
            training_directories << "person4/" << i << ".jpg";
        }
        directories = training_directories.str();
        images.push_back(imread(directories, CV_LOAD_IMAGE_GRAYSCALE)); labels.push_back(3);    
    }

    // images for fifth person
    for (int i = 0; i < 44; i++) {
        std::stringstream training_directories;
        if (i < 10) {
            training_directories << "person5/0" << i << ".jpg";
        } else {
            training_directories << "person5/" << i << ".jpg";
        }
        directories = training_directories.str();
        images.push_back(imread(directories, CV_LOAD_IMAGE_GRAYSCALE)); labels.push_back(4);    
    }
    
    Ptr<FaceRecognizer> model = createFisherFaceRecognizer();

    model->train(images,labels);

    return 0;
}