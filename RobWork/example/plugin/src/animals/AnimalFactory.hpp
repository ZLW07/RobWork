#ifndef __ANIMAL_FACTORY_HPP
#define __ANIMAL_FACTORY_HPP

#include <string>
#include <rw/core/ExtensionPoint.hpp>

#include "Animal.hpp"



namespace animals {

class AnimalFactory: public rw::core::ExtensionPoint<Animal>{
private:

	AnimalFactory();
	
public:

	/**
	 * Make new animals based on the species name.
	 * 
	 * 'cat' is available by default; other species are added by plugins.
	 */
	static Animal::Ptr getAnimal(const std::string& species);

};

}

#endif
