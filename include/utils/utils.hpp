#ifndef UTILS_HPP
#define UTILS_HPP

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

// #include <Eigen/Dense>

template<typename T>
std::vector<T> AsVector(boost::property_tree::ptree const & ptIn, boost::property_tree::ptree::key_type const & key)
{
  std::vector<T> r;
  for(auto & item : ptIn.get_child(key)) r.push_back(item.second.get_value<T>());
  return r;
}

// inline Eigen::VectorXd & quatVector(const Eigen::Quaterniond & input)
// {
//   Eigen::VectorXd output(4);
//   output(0) = input.w();
//   output(1) = input.x();
//   output(2) = input.y();
//   output(3) = input.z();
//   return output;
// }

// inline Eigen::Quaterniond & vecQuat(const Eigen::VectorXd & input)
// {

//   Eigen::Quaterniond output;
//   output.w() = input(0);
//   output.x() = input(1);
//   output.y() = input(2);
//   output.z() = input(3);

//   return output;
// }

#endif
