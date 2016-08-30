/*
 * Filereader.h
 *
 *  Created on: 19.08.2016
 *      Author: andreask
 */

#ifndef FILEREADER_H_
#define FILEREADER_H_

#include <iostream>
#include <fstream>

#include <vector>

#include <eigen3/Eigen/Dense>

class Filereader{
public:
	void readParameters(std::string filename, int index, std::vector<Eigen::Vector2d>& points2D_1);
};

#endif /* FILEREADER_H_ */
