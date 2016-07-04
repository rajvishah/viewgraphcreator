#include <vector>
#include <iostream>
#include <fstream>

#include <glog/logging.h>
#include <theia/theia.h>

#include "Reader.h"
#include "keys2a.h"

#include <stdio.h>

using namespace std;

int main(int argc, char* argv[]) {
  // Initialize logging to use --log_dir flag
  google::InitGoogleLogging(argv[0]);

  // Input files
  // argv[1] stores path to list files and matches
  string imgListFile = string(argv[1]) ;
  string keyListFile = string(argv[1]) ;
  string matchesFile = string(argv[1]) + "/matches.txt";

  // Output files 
  string connectedPairsFile = string(argv[1]) + "/connected-pairs.txt";
  string intrinsicFileName = string(argv[1]) + "/intrinsics.txt"; 
  string viewgraphFile = string(argv[1]) + "/view-graph.bin";

  // Read image file names and dimensions
  reader::ImageListReader imgList(imgListFile);
  bool s1 = imgList.read();

  // Read keyfile names
  reader::KeyListReader keyList(keyListFile);
  bool s2 = keyList.read();

  // Check if list files are read correctly
  if(!s1 || !s2) {
    printf("\nError reading list file or key file");
    return 0;
  }

  // Open pairs output file 
  ofstream pairsFile( connectedPairsFile.c_str(), ofstream::out); 
  if(!pairsFile.is_open()) {
    cout << "\nError opening pairs file";
    return -1;
  }
  
  // Initialize containers for various data
  int numKeys = keyList.getNumKeys();
  vector< unsigned char* > keys(numKeys);
  vector< keypt_t* > keysInfo(numKeys);
  vector<theia::KeypointsAndDescriptors> kpdVec(numKeys);

  vector< int > numFeatures(numKeys);
  vector<string> view_names(numKeys);

  vector<double> halfWidth(numKeys);
  vector<double> halfHeight(numKeys);
  vector<double> focalLengths(numKeys, 1.0f);
  vector<bool> calibrationFlag(numKeys, false);

  vector<theia::CameraIntrinsicsPrior> camPriors;

  //Create a single ExifReader object to extract focal lengths
  theia::ExifReader exReader;
  for(int i=0; i < keyList.getNumKeys(); i++) {
    printf("\nReading keyfile %d/%d", i, numKeys);
    string keyPath = keyList.getKeyName(i);
    numFeatures[i] = ReadKeyFile(keyPath.c_str(),
        &keys[i], &keysInfo[i]);
    int W = imgList.getImageWidth(i);
    int H = imgList.getImageHeight(i);
    NormalizeKeyPoints(numFeatures[i], keysInfo[i], W, H);

    theia::KeypointsAndDescriptors& kpd = kpdVec[i];
    kpd.image_name = imgList.getImageName(i);
    kpd.keypoints.resize(numFeatures[i]);
    kpd.descriptors.resize(numFeatures[i]);

    for(int n=0; n < numFeatures[i]; n++) {
      double x1 = keysInfo[i][n].x;
      double y1 = keysInfo[i][n].y;
      theia::Keypoint kp(x1, y1, theia::Keypoint::SIFT);
      kpd.keypoints[n] = kp;

      kpd.descriptors.resize(128);
      Eigen::VectorXf desc(128);
      for(int d=0; d < 128; d++) {
        desc[d] = *(keys[i] + (128*n) +d); 
      }
    }

    printf("\nRead key file");
    fflush(stdout);
    halfWidth[i] = (double)W/2.0f;
    halfHeight[i] = (double)H/2.0f;

    // Get image name (abc.jpg) if full path is given
    string path = imgList.getImageName(i);
    size_t sep = path.find_last_of("\\/");
    if (sep != std::string::npos)
      path = path.substr(sep + 1, path.size() - sep - 1);
    view_names[i] = path; 

    // Read EXIF data find focal length 
    printf("\nReading exif data %d/%d - %s", i, numKeys, imgList.getImageName(i).c_str());
    theia::CameraIntrinsicsPrior cPrior, dummyPrior;
    bool status = exReader.ExtractEXIFMetadata(imgList.getImageName(i), &cPrior);
    if(status) {
      printf("\nRead EXIF data:");
      if(cPrior.focal_length.is_set) {
        calibrationFlag[i] = true;
        focalLengths[i] = cPrior.focal_length.value;
      } 
    }
    
    if(!calibrationFlag[i]) { /*Either ExifReader returned false or focal length was not set*/
      printf("\nUsing median focal length for image %d", i);
      int maxDim = W > H ? W : H;
      double focalGuess = 1.2*maxDim;
      calibrationFlag[i] = true;
      focalLengths[i] = focalGuess;

      cPrior.focal_length.is_set = true;
      cPrior.focal_length.value = focalLengths[i];
      cPrior.image_width = W;
      cPrior.image_height = H;
    } 


    /// Rajvi - I am making this change to avoid normalization effect issue while track formation
    cPrior.principal_point[0].is_set = true;
    cPrior.principal_point[1].is_set = true;
    cPrior.principal_point[0].value = halfWidth[i];
    cPrior.principal_point[1].value = halfHeight[i];
    /////////////////////////////////////////////////////////  */

    camPriors.push_back(cPrior);


    printf("\nRead exif file");
    fflush(stdout);
  }



  // Read matches file and veriy two view matches
  FILE* fp = fopen( matchesFile.c_str() , "r");
  if( fp == NULL ) {
    printf("\nCould not read matches file");
    fflush(stdout);
    return 0;
  }


  int img1, img2;
  vector<theia::ImagePairMatch> matches;
  int numPairsVerified = 0, numPairsTotal=0;

  // Read image indices
  while(fscanf(fp,"%d %d",&img1,&img2) != EOF) {

    int numMatches;
    fscanf(fp,"%d",&numMatches);

    numPairsTotal++;

    vector< pair<int,int> > matchIdxPairs;
    vector<theia::FeatureCorrespondence> featCorrs;
    vector<theia::FeatureCorrespondence> inlFeatCorrs;


    vector<theia::IndexedFeatureMatch> mgTheia;
    for(int i=0; i < numMatches; i++) {
      int matchIdx1, matchIdx2;
      fscanf(fp,"%d %d",&matchIdx1, &matchIdx2);

      ///////////// Rajvi supply normalized for test //////////
      float x1 = keysInfo[img1][matchIdx1].nx;
      float y1 = keysInfo[img1][matchIdx1].ny; 

      float x2 = keysInfo[img2][matchIdx2].nx; 
      float y2 = keysInfo[img2][matchIdx2].ny;

      // printf("\nView1 %d, sift1 %d, x %lf y %lf", img1, matchIdx1, x1, y1);
      // printf("\nView2 %d, sift2 %d, x %lf y %lf", img2, matchIdx2, x2, y2);
      ///////////// Rajvi supply unnormalized for test //////////

      // float x1 = keysInfo[img1][matchIdx1].x;
      // float y1 = keysInfo[img1][matchIdx1].y; 

      // float x2 = keysInfo[img2][matchIdx2].x; 
      // float y2 = keysInfo[img2][matchIdx2].y; 


      theia::IndexedFeatureMatch m1( matchIdx1, matchIdx2, 1.0);
      mgTheia.push_back(m1);

      theia::Feature f1(x1, y1);
      theia::Feature f2(x2, y2);

      theia::FeatureCorrespondence corres;
      corres.feature1 = f1;
      corres.feature2 = f2;

      featCorrs.push_back(corres);     

      matchIdxPairs.push_back(make_pair(matchIdx1, matchIdx2));
    }

    vector<int> inliers;

    theia::TwoViewInfo currPairInfo;
    theia::TwoViewMatchGeometricVerification::Options options;
    options.min_num_inlier_matches = 30;

    theia::TwoViewMatchGeometricVerification twoViewVer(options,
        camPriors[img1], camPriors[img2],
        kpdVec[img1],kpdVec[img2],
        mgTheia);

    vector<theia::FeatureCorrespondence> verified_matches; 
    bool status = twoViewVer.VerifyMatches(&verified_matches, &currPairInfo);
    if(status) {
      numPairsVerified++;
      pairsFile << img1 << " " << img2 << endl;

      theia::ImagePairMatch imPair;
      imPair.image1 = view_names[img1]; /*imgList.getImageName(img1);*/
      imPair.image2 = view_names[img2]; /*imgList.getImageName(img2); */

      imPair.twoview_info = currPairInfo;
      imPair.correspondences = verified_matches;
      matches.push_back(imPair);
    }
  }


  pairsFile.close();
  bool status = WriteMatchesAndGeometry(viewgraphFile, view_names, camPriors, matches);
  if(status) {
    printf("\nSuccessfully written view-graph file");
  }
  printf("\n%d of %d pairs verified", numPairsVerified, numPairsTotal);
  fflush(stdout);
  return 0;
}
