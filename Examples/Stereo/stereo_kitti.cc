/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    /*
    struct shm_remove
              {
                shm_remove() { boost::interprocess::shared_memory_object::remove("MySharedMemory"); }
                ~shm_remove(){ boost::interprocess::shared_memory_object::remove("MySharedMemory"); }
              } remover;
    */
    if(argc != 4)
    {
        cerr << endl << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    
    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), vstrImageLeft, vstrImageRight, vTimestamps);

    const int nImages = vstrImageLeft.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::STEREO,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    

    //if enabled the SLAM program will rather get images from Network than from disk.
    int frames_processed = 0;
    //vector<char*> buffImageLeft;
    //vector<char*> buffImageRight;
    
    /*
    if(GET_IMAGE_FROM_NETWORK){
        getImageNetwork(&SLAM, buffImageLeft, vTimesTrack);
    }
    else{
    */

    //resize time tracker
    vTimesTrack.resize(nImages);  

    // Main loop
    cv::Mat imLeft, imRight;
    int count_images = 0;

    std::chrono::steady_clock::time_point Start_frame = std::chrono::steady_clock::now();
    double time_for_postload;
    for(int ni=0; ni<nImages; ni++)
    {

        //stop after 100 frames.
        count_images++;
        if(count_images==100)
        {
            std::cout<<" --- More than 100 images --- we pause now and run postload "<<std::endl<<std::endl;
            std::cout<<"------------------------------------------------------------"<<std::endl;
            std::cout<<"------------------------------------------------------------"<<std::endl;
            std::cout<<"------------------------------------------------------------"<<std::endl;
            std::cout<<"------------------------------------------------------------"<<std::endl;
            std::chrono::steady_clock::time_point postLoad_start = std::chrono::steady_clock::now();
            SLAM.PostLoad(); //run the post load function
            std::chrono::steady_clock::time_point postLoad_end = std::chrono::steady_clock::now();
            time_for_postload= std::chrono::duration_cast<std::chrono::duration<double> >(postLoad_end - postLoad_start).count();
            //int flag = std::cin.get();
            // Stop all threads
            //SLAM.Shutdown();
            // Save camera trajectory
           //SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");
           //int hold;
           //std::cin>>hold;
           std::cout<<std::endl<<std::endl;
           std::cout<<"------------------------------------------------------------"<<std::endl;
           std::cout<<"------------------------------------------------------------"<<std::endl;
           std::cout<<"------------------------------------------------------------"<<std::endl;
           std::cout<<"------------------------------------------------------------"<<std::endl;
           std::cout<<"------------------------------------------------------------"<<std::endl;
        }
        else{
            std::cout<<"@@@@ Counted till: "<<count_images<<" images.\n";
        }

        
        if(count_images>=1999)
        {
            std::cout<<"1999 frames done\n";
            std::chrono::steady_clock::time_point End_frame_n = std::chrono::steady_clock::now();
            double total_time_process_n = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(End_frame_n - Start_frame).count();
            std::cout<<"Time taken for this process: "<<total_time_process_n<<"ms\n Time for PostLoad: "<<time_for_postload<<"ms \n";
            std::cout<<"==== @@@@ ==== Exiting. Before shutdown. ==== @@@@ ==== \n";
            int aa;
            std::cin>>aa;
            SLAM.Shutdown();
            SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");
            exit(0);
        }
        
        
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni],cv::IMREAD_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeft,imRight,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;
	//Aditya's metric
	cout<<"Tracking time: "<<ttrack<<endl;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    std::chrono::steady_clock::time_point End_frame = std::chrono::steady_clock::now();
    double total_time_process = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(End_frame - Start_frame).count();
    std::cout<<"Time taken for this process: "<<total_time_process<<"ms\n Time for PostLoad: "<<time_for_postload<<"ms \n";

//} //The main processing loop without network ends here.

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

    return 0;
}

void getImageNetwork(ORB_SLAM3::System *SLAM, int frames_processed, vector<float> vTimesTrack){
    /*
    boost::circular_buffer<char*> leftImages(1000);
    boost::circular_buffer<char*> rightImages(1000);
    // For the network receive:
    try{
        boost::asio::io_service ioService;
        Server server(ioService, 8080,leftImages,rightImages);
        //ioService.run(); //use poll option when polling this.
    } catch(std::exception& e){
        std::cerr<<"Network Exception: "<<e.what()<<"\n";
    }
    // Main loop
    cv::Mat imLeft, imRight;

    //run the polling loop.
    while(1){
        //Poll the network connection
        ioService.poll();

        //check if there are new images.
        if(!leftImages.empty()){
            imLeft = cv::imdecode(leftImages.pop_front(),cv::IMREAD_UNCHANGED,imLeft);
            
        }

    }

    //Run the openCV decode before 

 */
    return;

}

/*
void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}

*/
void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    int counter = 0;
    int skip = 10000; //skip one in 300
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            if(counter%skip != 0){
                vTimestamps.push_back(t);
            }
            else
                std::cout<<"SKIPPED\n";
            counter++;
            if (counter==200){
                break;
            }
        }

    }
    std::cout<<"Counter: "<<counter<<std::endl;
    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = vTimestamps.size();
    const int nTimes_loop = counter;
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    std::cout<<"---- Before getting in the loop \n";
    int new_counter = 0;
    //for(int i=0; i<nTimes_loop; i++)
    for(int i=2300; i<2500; i++)
    {
        if(i%skip !=0){
            stringstream ss;
            /*
            ss << setfill('0') << setw(6) << i;
            vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
            vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
            */
            ss << setfill('0') << setw(6) << i;
            vstrImageLeft[new_counter] = strPrefixLeft + ss.str() + ".png";
            vstrImageRight[new_counter] = strPrefixRight + ss.str() + ".png";
            new_counter++;
        }
        else
            std::cout<<"2nd SKIPPED\n";

    }   
}
