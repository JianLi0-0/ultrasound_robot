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

class CustomTimer
{
	public:
    	CustomTimer(){};
    	~CustomTimer(){};
		void tic()
		{
			start = std::chrono::system_clock::now();
		}
		double toc(bool print)
		{
			elapsed_seconds = std::chrono::system_clock::now() - start;
			if(print) {std::cout << "elapsed time: " << elapsed_seconds.count() << std::endl;}
			return elapsed_seconds.count();
		}
	private:
		    std::chrono::time_point<std::chrono::system_clock> start;
    		std::chrono::duration<double> elapsed_seconds;
};

#endif
