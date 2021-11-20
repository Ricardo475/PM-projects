
#include "calibration_node.h"

void read_chess_info()
{

  std::ifstream chessFile;
  std::string dist = "Dist";
  std::string x = "X";
  std::string y = "Y";
  std::string aux = "";

  chessFile.open("catkin_ws/src/assignment1/camara_calibration/Chess_info.txt");

  if(chessFile.is_open()){

    while(!chessFile.eof()){

      getline(chessFile,str);

      if(str.find(dist) != std::string::npos){

        for(auto n: str){

          if(n == ' '){
            std::cout << "DIST: " + aux << std::endl;
            chess_dist = stof(aux);
            break;
          }
          else
            aux = aux + n;
        }
        std::cout << "DIST final: " + std::to_string(chess_dist) << std::endl;;
      }

      else if(str.find(x) != std::string::npos){

        for(auto n: str){

          if(n == ' '){
            std::cout << "X: " + aux << std::endl;;
            chess_x = stoi(aux);
            break;
          }
          else
            aux = aux + n;
        }
        std::cout << "X final: " + std::to_string(chess_x) << std::endl;;
      }

      else if(str.find(y) != std::string::npos){

        for(auto n: str){

          if(n == ' '){
            std::cout << "Y: " + aux << std::endl;
            chess_y = stoi(aux);
            break;
          }
          else
            aux = aux + n;
        }
        std::cout << "Y final: " + std::to_string(chess_y) << std::endl;;
      }

      aux = "";
    }

  }

}

void create_XML(cv::Mat K, cv::Vec<float, 5> k){

  std::string matrixK_rows[] = {"[", "[", "["};
  std::vector<float> aux1;
  float aux2;
  for(int i = 0; i < K.rows;i++){
    aux1 = K.row(i);
   for(int j = 0; j < int(aux1.size());j++){

     if(j == 0)
       matrixK_rows[i] += std::to_string(float(aux1.at(j)));
     else
       matrixK_rows[i] += ", " + std::to_string(float(aux1.at(j)));
   }
      matrixK_rows[i] += "]\n";
  }

  std::string matrixk = "[";

  for(int i = 0; i < k.rows; i++)
  {
    aux2 = k.val[i];

    if(i==0)
      matrixk += std::to_string(float(aux2));
    else
      matrixk += ", " + std::to_string(float(aux2));
  }

  matrixk += "]\n";
  std::cout << "k :" << matrixk;
  std::string xml_file = "<?xml version=\"1.0\"?>\n\
<Camera_calibration>\n\
  <name>intrinsic_matrix\n\
    <description>Intrinsic matrix k result</description>\n\
    <matrix>\n\
      " + matrixK_rows[0] + "\
      " + matrixK_rows[1] + "\
      " + matrixK_rows[2] + "\
    </matrix>\n\
  </name>\n\
  <name>lens_distortions\n\
    <description>Lens distortion coefficients</description>\n\
    <vector>\n\
      " + matrixk + "\
    </vector>\n\
   </name>\n\
</Camera_calibration>";


  std::cout << xml_file << std::endl;

  std::ofstream xmlFile( "catkin_ws/src/assignment1/camara_calibration/camara_calibration_info.xml");
  xmlFile << xml_file;
  xmlFile.close();

}

void camera_calibration(){

  std::vector<cv::String> fileNames;
  cv::glob("catkin_ws/src/assignment1/camara_calibration/images/WIN*.jpg",fileNames,false);
  cv::Size patternSize(chess_x-1, chess_y-1);
  std::vector<std::vector<cv::Point2f>> q(fileNames.size());

  std::vector<std::vector<cv::Point3f>> Q;

  int chessBoard[2] = {chess_x,chess_y};
  int squareSize = int (chess_dist*1000);

  std::vector<cv::Point3f> objp;
  for(int i = 1; i <chessBoard[1];i++){
    for(int j = 1; j<chessBoard[0];j++)
      objp.push_back(cv::Point3f(j*squareSize,i*squareSize,0));
  }


  std::vector<cv::Point2f> imgPoint;
  std::size_t i = 0;
  for(auto const &file: fileNames){

    cv::Mat img = cv::imread(file);
    cv::Mat gray;

     cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);

     bool patternFound = cv::findChessboardCorners(gray, patternSize, q[i], cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_FAST_CHECK);

     if(patternFound){
             cv::cornerSubPix(gray, q[i],cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
             Q.push_back(objp);
         }
     else{
       std::cout << "PATTERN NOT FOUND!\n";
     }

     cv::drawChessboardCorners(img, patternSize, q[i], patternFound);
        // cv::imshow("chessboard detection", img);
        // cv::waitKey(0);

     i++;
  }

cv::Mat K(cv::Matx33f::eye());
cv::Vec<float, 5> k(0, 0, 0, 0, 0);

std::vector<cv::Mat> rvecs, tvecs;
  std::vector<double> stdIntrinsics, stdExtrinsics, perViewErrors;
  int flags = cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_K3 +
              cv::CALIB_FIX_PRINCIPAL_POINT;
  cv::Size pixelSize(1280, 720);

   std::cout << "Doing calibration..." << std::endl;


   float error = float (cv::calibrateCamera(Q, q, pixelSize, K, k, rvecs, tvecs, flags));

   std::cout << "Error = " << error << "\nK =\n"
             << K << "\nk=\n"
             << k << std::endl;
   // Precompute lens correction interpolation
   cv::Mat mapX, mapY;
   cv::initUndistortRectifyMap(K, k, cv::Matx33f::eye(), K, pixelSize, CV_32FC1,
                               mapX, mapY);

   // Show lens corrected images
   for (auto const &f : fileNames) {
     std::cout << std::string(f) << std::endl;

     cv::Mat img = cv::imread(f, cv::IMREAD_COLOR);

     cv::Mat imgUndistorted;
     // 5. Remap the image using the precomputed interpolation maps.
     cv::remap(img, imgUndistorted, mapX, mapY, cv::INTER_LINEAR);

     // Display
    // cv::imshow("undistorted image", imgUndistorted);
    // cv::waitKey(0);
   }

   create_XML(K,k);
}


int main(int argc, char **argv)
{
  read_chess_info();
  camera_calibration();
  return 0;
}
