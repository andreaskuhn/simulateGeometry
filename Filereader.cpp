/*
 * Filereader.cpp
 *
 *  Created on: 19.08.2016
 *      Author: andreask
 */

#include "Filereader.h"

void
Filereader::readParameters(std::string filename, int index, std::vector<Eigen::Vector2d>& points2D_1){
	std::cerr << "Reading in file: " << filename << "..." << std::endl;
	std::ifstream inStream(filename.c_str());
	if(!inStream.good()){
		std::cout << "error in readCamConfig: file " << filename << " not found" << std::endl;
		return;
	}
	std::stringstream ss;
	ss << index;
	std::string searchstring = ss.str()+".";
	while(inStream.good()){
		std::string buffer;
		getline(inStream, buffer, '\n');
		if(buffer.compare(searchstring)==0){
			std::cout << "FOUND" << std::endl;
			while(inStream.good()){
				getline(inStream, buffer, '\n');
				if(buffer.size()<1)
					return;
				int pos = buffer.find(" ", 0);
				int pos2 = buffer.find(" ", pos+1);
				std::stringstream stm1(buffer.substr(0, pos));
				std::stringstream stm2(buffer.substr(pos+1, pos2-pos));
				std::cout << buffer << std::endl;
				//std::cout << stm1.str() << std::endl;
				//std::cout << stm2.str() << std::endl;
				Eigen::Vector2d temp;
				stm1 >> temp[0];
				stm2 >> temp[1];
				points2D_1.push_back(temp);
			}
			return;
		}
	}
}

