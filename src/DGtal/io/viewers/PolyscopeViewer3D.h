/**
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 **/
#pragma once

#include <map>
#include <set>
#include <iostream>
#include <functional>
#include "polyscope/polyscope.h"

#include "polyscope/curve_network.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/volume_mesh.h"
#include "polyscope/volume_grid.h"
#include "polyscope/point_cloud.h"
#include "polyscope/pick.h"


#include "DGtal/io/Display3D.h"
#include "DGtal/io/Display3DFactory.h"

/**
 * @file PolyscopeViewer3D.h
 * @author Bastien Doignies
 *
 * @date 2025/03/20
 *
 * This file is part of the DGtal library.
 */

namespace DGtal
{
    /**
     * Description of class 'PolyscopeViewer3D' <p>
     * Aim: Display 3D primitive (like PointVector, DigitalSetBySTLSet,
     * Object ...). This class uses the polyscope library (@see
     * https://polyscope.run). It inherits of the class Display3D
     * and displays objects using a simple stream mechanism of
     * "<<".
     *
     * For instance you can display objects as follows:
     *
     * @code
     * #include "DGtal/helpers/StdDefs.h"
     * #include "DGtal/io/viewers/PolyscopeViewer3D.h"
     * ...
     * using namespace Z3i;
     * ...
     *
     * Point p1( 0, 0, 0 );
     * Point p2( 5, 5 ,5 );
     * Point p3( 2, 3, 4 );
     * Domain domain( p1, p2 );
     * PolyscopeViewer3D<> viewer;
     * viewer << domain;
     *  viewer << p1 << p2 << p3;
     * viewer<< Viewer3D<>::updateDisplay;
     * viewer.show();
     *
     * @endcode
     *
     * This class is parametrized by both the Digital and Khalimsky
     * space used to display object. More precisely, embed methods are
     * used to compute the Euclidean coordinate of digital
     * objects/khalimksy cells.
     *
     * @tparam TSpace any model of Digital 3D Space
     * @tparam TKSpace any mode of Khalimksky 3D space
     *
     * @note This class perform polyscope initialization. 
     *
     * @note When possible this class tries to register polyscope structures under the provided name (@see DrawWithDisplay3DModifier::SetName3D). If not possible, it will register them under "{prefix}_{idx}". 
     *
     * @note You *must* provide a Khalimksy space at instanciation if
     * you wish to display cells with the viewer. If you are not going
     * to display cells, then it is not compulsory to provide it.
     *
     * @see Display3D, Board3DTo2D
     */
    template <typename TSpace  = SpaceND<3>,
              typename TKSpace = KhalimskySpaceND<3>>
    class PolyscopeViewer3D : public Display3D<TSpace, TKSpace>
    {
    public:
        /**
         * Interface that can be used so that one can extend a few service
         * of PolyscopeViewer3D, like keyPressEvent and others. You may thus give an
         * extension to a PolyscopeViewer3D by simply handling it a pointer to an
         * object deriving from this class.
         */
        struct Extension 
        {
        public:
            /**
             * Main callback to handle UI and callback. Use ImGUI to handles input and setup callbacks. 
             */
            virtual void UICallback() {};

            /**
             * Main callback to handle click and selections.
             *
             * @param viewer The viewer that calls this function
             * @param structure The polyscope structure that is clicked
             * @param structureIndex Index within \p structure
             * @param dgtalName DGtal name of the clicked element
             */
            virtual void OnSelect(
                PolyscopeViewer3D<TSpace, TKSpace>* viewer,

                polyscope::Structure* structure, 
                uint32_t structureIndex, 
                DGtal::int32_t dgtalName
            ) {};
        };
        
        /**
        * Default constructor
        *
        * Intializes polyscope
        */
        PolyscopeViewer3D();

        /**
         * Constructor from Khalimsky space
         *
         * Initializes polyscope
         *
         * @param emb The khalimsky space
         */
        PolyscopeViewer3D(const TKSpace& emb);

        /**
         * Main function to control the viewer
         *
         * @param key The command
         */
        PolyscopeViewer3D<TSpace, TKSpace> & operator<<(
            const typename PolyscopeViewer3D<TSpace, TKSpace>::StreamKey & key
        );
        
        /**
         * Add an object to draw list
         *
         * @param object The object to draw
         */
        template <typename TDrawableWithViewer3D>
        PolyscopeViewer3D<TSpace, TKSpace> & operator<<(const TDrawableWithViewer3D & object);

        /**
         * Set the name of further object to be drawn.
         *
         * @param name The name
         */
        PolyscopeViewer3D<TSpace, TKSpace>& operator<<(const SetName3D& name);
        
        /**
         * Show the viewer and start the event loop. Must be called only once. 
         */
        void show() const;

        /**
         * Add a message to the list
         *
         * @param message the message
         */
        void displayMessage(const std::string& message);
        
        /**
         * Writes/Displays the object on an output stream.
         * 
         * @param out The output stream
         */
        void selfDisplay( std::ostream & out ) const;
        
        /**
         * Set the extension. Does not take ownership.
         *
         * @param ext The extension
         */
        void setExtension(Extension* ext);
        
        /**
         * Set the extension. 
         *
         * @param ext The extension
         */
        void setExtension(CountedPtr<Extension> ext);

    public: // Reimplement from Display3D

        /**
         * Add a clipping plane by its normal and offset
         *
         * @param a normal x component
         * @param b normal y component
         * @param c normal z component
         * @param d plane offset
         * @param drawPlane If plane should be drawn. Unused for this class
         */
        void addClippingPlane(double a, double b, double c, double d, bool drawPlane) override;
    protected:
        /**
         * Polyscope UI callback
         */
        virtual void poyscopeCallback();

    private:
        /**
         * Clear object and names
         */
        void clearAll();
        
        /** 
         * Create and register polyscope structures
         *
         * @see registerClippingPlanes
         * @see registerTriangles
         * @see registerCubeMaps
         * @see registerPolygons
         * @see registerLines
         * @see registerBalls
         * @see registerQuads
         * @see registerImages
         */
        void createPolyscopeObjects() const;
        
        /** Register clipping planes */
        void registerClippingPlanes() const;
        /** Register triangles */
        void registerTriangles() const;
        /** Register cubes */
        void registerCubeMaps() const;
        /** Register polygons */
        void registerPolygons() const;
        /** Register lines */
        void registerLines() const;
        /** Register balls (points) */
        void registerBalls() const;
        /** Register quads */
        void registerQuads() const;

        /** Register images */
        void registerImages() const;

    private:
        /// The extension to call upon events
        DGtal::CountedPtr<Extension> extension;
        /// Queue of messages to be displayed
        std::vector<std::string> messageQueue;

        // Necessary to bind callback to appropriate data
        // Names are unique but not indices, hence we can't use a bimap..
    
        /**
         * Maps structure index to DGtal name
         */
        struct IndexMapper
        {
            // Track if user has specified the name or if it was
            // implictely given. 
            bool userSpecified = false;
            // Map
            std::map<DGtal::int32_t, DGtal::int32_t> indexMap;
        };
        /// Mapping from polyscope names to DGtal names
        mutable std::map<std::string, IndexMapper> namesMap;
    };  

    template <typename TSpace, typename TKSpace>
    std::ostream & operator<<(std::ostream & out, const PolyscopeViewer3D<TSpace, TKSpace> & object);
}

#include "PolyscopeViewer3D.ih"
