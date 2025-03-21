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
 * Header file for concept Polyscopeviewer.cpp
 *
 * This file is part of the DGtal library.
 */

namespace DGtal
{
    template <typename TSpace  = SpaceND<3>,
              typename TKSpace = KhalimskySpaceND<3>>
    class PolyscopeViewer3D : public Display3D<TSpace, TKSpace>
    {
    public:
        struct Extension 
        {
        public:
            virtual void UICallback() {};
            virtual void OnSelect(
                PolyscopeViewer3D<TSpace, TKSpace>* viewer,

                polyscope::Structure* structure, 
                uint32_t structureIndex, 
                DGtal::int32_t voxelName
            ) {};
        };

        PolyscopeViewer3D();


        PolyscopeViewer3D<TSpace, TKSpace> & operator<<(
            const typename PolyscopeViewer3D<TSpace, TKSpace>::StreamKey & key
        );

        template <typename TDrawableWithViewer3D>
        PolyscopeViewer3D<TSpace, TKSpace> & operator<<(const TDrawableWithViewer3D & object);

        PolyscopeViewer3D<TSpace, TKSpace>& operator<<(const SetName3D& name);
        

        void show() const;
        void displayMessage(const std::string& message);
        void selfDisplay( std::ostream & out ) const;

        void setExtension(Extension* ext);
        void setExtension(CountedPtr<Extension> ext);

    public: // Reimplement from Display3D

        // Avoid adding quads
        void addClippingPlane(double a, double b, double c, double d, bool drawPlane) override;
    protected:
        virtual void poyscopeCallback();

    private:
        void clearAll();

        void createPolyscopeObjects() const;

        void registerClippingPlanes() const;
        void registerCubeMaps() const;
        void registerLines() const;
        void registerBalls() const;
        void registerQuads() const;


    private:
        DGtal::CountedPtr<Extension> extension;

        std::vector<std::string> messageQueue;

        // Necessary to bind callback to appropriate data
        // Names are unique but not indices, hence we can't use a bimap...
        
        struct IndexMapper
        {
            // Track if user has specified the name or if it was
            // implictely given by the balls.
            bool userSpecified = false;
            std::map<DGtal::int32_t, DGtal::int32_t> indexMap;
        };
        mutable std::map<std::string, IndexMapper> namesMap;
    };  

    template <typename TSpace, typename TKSpace>
    std::ostream & operator<<(std::ostream & out, const PolyscopeViewer3D<TSpace, TKSpace> & object);
}

#include "PolyscopeViewer3D.ih"