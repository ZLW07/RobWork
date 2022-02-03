#ifndef RWS_RWSIMAGELOADERPLUGIN_HPP_
#define RWS_RWSIMAGELOADERPLUGIN_HPP_

#include <rw/core/Extension.hpp>
#include <rw/core/Plugin.hpp>
#include <rw/core/PropertyMap.hpp>
#include <rw/core/Ptr.hpp>

namespace rws {

/**
 * @brief A RobWork image loader factory plugin. It adds additional image loader functionality
 * to the rw::loaders::ImageFactory through RobWork plugin structure.
 */
class RWSImageLoaderPlugin : public rw::core::Plugin
{
  public:
    RWSImageLoaderPlugin ();

    virtual ~RWSImageLoaderPlugin ();

    std::vector< rw::core::Extension::Descriptor > getExtensionDescriptors ();

    rw::core::Ptr< rw::core::Extension > makeExtension (const std::string& str);

  private:
    rw::core::PropertyMap _map;
};

}    // namespace rws

#endif /* RWSIMAGELOADERPLUGIN_HPP_ */
