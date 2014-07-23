#include "generateList.h"

using namespace std;


void generateMarcelTrainLists() {
    generateList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Train/A/","A-train",1,1329,".ppm", "1");
    generateList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Train/B/","B-train",1,487,".ppm", "2");
    generateList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Train/C/","C-train",1,572,".ppm", "3");
    generateList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Train/Five/","Five-train",1,654,".ppm", "4");
    generateList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Train/Point/","Point-train",1,1395,".ppm", "5");
    generateList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Train/V/","V-train",1,435,".ppm", "6");
}

void generateMarcelTestLists() {
    // 39 41 47 58 54 38
    generateList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Test/A/complex/","A-complex",1,39,".ppm", "1");
    generateList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Test/B/complex/","B-complex",1,41,".ppm", "2");
    generateList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Test/C/complex/","C-complex",1,47,".ppm", "3");
    generateList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Test/Five/complex/","Five-complex",1,58,".ppm", "4");
    generateList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Test/Point/complex/","Point-complex",1,54,".ppm", "5");
    generateList("/home/gmanfred/devel/essential/cpp/swift_hand/data/Marcel-Test/V/complex/","V-complex",1,38,".ppm", "6");
}

void generateList(string base_folder, string base_name, int start_idx, int end_idx, string ext, string label) {
    ofstream file;
    string list_path = base_folder + "/list.txt";
    file.open(list_path.c_str());
    for (int i = start_idx; i <= end_idx; ++i) {
        if (i < 10)
            file << base_folder << base_name << "000" << i << ext << " " << label << endl;
        else if (i < 100)
            file << base_folder << base_name << "00" << i << ext << " " << label << endl;
        else if (i < 1000)
            file << base_folder << base_name << "0" << i << ext << " " << label << endl;
        else
            file << base_folder << base_name << i << ext << " " << label << endl;
    }
    file.close();
}

