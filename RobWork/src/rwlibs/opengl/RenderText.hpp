#ifndef RWLIBS_OPENGL_RENDERTEXT_HPP
#define RWLIBS_OPENGL_RENDERTEXT_HPP

/**
 * @file RenderText.hpp
 */

#include <RobWorkConfig.hpp>
#include <rw/core/Ptr.hpp>
#include <rw/graphics/Render.hpp>
#include <rwlibs/opengl/SceneOpenGL.hpp>
#include <rwlibs/os/rwgl.hpp>

#include <string>
#include <vector>

namespace rwlibs { namespace opengl {

    class RenderText : public rw::graphics::Render
    {
      public:
        /**
         * @brief Constructs a RenderText
         * @param text [in] the text to be rendered
         */
        RenderText (std::string text, rw::core::Ptr< rw::kinematics::Frame > frame);

        //! @copydoc rw::graphics::Render::draw(const DrawableNode::RenderInfo& info,
        //! DrawableNode::DrawType type, double alpha) const
        void draw (const rw::graphics::DrawableNode::RenderInfo& info,
                   rw::graphics::DrawableNode::DrawType type, double alpha) const;

      protected:
      private:
        std::string _text;
        rw::core::Ptr< rw::kinematics::Frame > _frame;
        bool _haveGlut;

        // Text width and text height in pixels
        int _textWidth;
        int _textHight;

        void* _font;

        void findTextDimensions ();
        std::vector< rw::math::Vector3D<> > getLabelCorners (rw::math::Transform3D<> fTc,
                                                             rw::math::Vector3D<> pos,
                                                             double scale_x, double scale_y,
                                                             int move_x, int move_y) const;
    };

}}        // namespace rwlibs::opengl
#endif    // end include guard