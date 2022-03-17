import os
import sys

if __name__=="__main__":

    #interval = int(sys.argv[2])
    #print(sys.argv[sys.argc-1])


    timeStampFile_path = '/home/adhak001/dev_slam/slam_fork/ORB_SLAM3/evaluation/'


    resultSava_path = '/home/adhak001/dev_slam/slam_fork/ORB_SLAM3/evaluation/'
    if not os.path.exists(resultSava_path):
        os.makedirs(resultSava_path)
    #resultSava_fileName = "00.txt"

    file = sys.argv[1]
    timestampfile = sys.argv[2]

    #interval = 50

    #write the resulting files


    resultSava_file = open(resultSava_path + file, "w+")
    #timeStampFile = open(timeStampFile_path+file, "r")
    timeStampFile = open(timestampfile, "r")
    timeStampFileString = timeStampFile.read()
    timeStampFileLines = timeStampFileString.split("\n")
    timeStampFileLines = filter(None, timeStampFileLines)
    #stamp are the starting and ending stamp in the sourceFile [stamp[0], stamp[1]]
    for i in range(0, len(timeStampFileLines)):
        #print(timeStampFileLines)
        x = timeStampFileLines[i].split(" ")[3]
        y = timeStampFileLines[i].split(" ")[11]
        z = timeStampFileLines[i].split(" ")[7]
        print(x + " " + y + " " + z)
        resultSava_file.write(str(i).zfill(6) + " " + x + " " + y + " " + z + "\n")





    resultSava_file.close()
    timeStampFile.close()

