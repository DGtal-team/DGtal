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
#include "DGtal/geometry/curves/estimation/LambdaMST3DBy2D.h"
/**
 * @file LambdaMST3DBy2D.cpp
 * @author Kacper Pluta (\c kacper.pluta@esiee.fr )
 * Laboratoire d'Informatique Gaspard-Monge - LIGM, France
 *
 * @date 2015/06/16
 *
 * This file is part of the DGtal library.
 */
using namespace DGtal;

TangentFromDSS3DBy2DFunctor::Vector3D TangentFromDSS3DBy2DFunctor::operator() ( TangentFromDSS3DBy2DFunctor::MAIN_AXIS mainAxis, const TangentFromDSS3DBy2DFunctor::Vector2D & v0, const TangentFromDSS3DBy2DFunctor::Vector2D & v1 ) const
{
  TangentFromDSS3DBy2DFunctor::Vector3D tangent;
  if ( mainAxis == X )
  {
    if ( v1[0] == 0 || ( v0[1] == 0 && v1[1] == 0 ) )
    {
      tangent[0] = v0[1];
      tangent[1] = v0[0];
      tangent[2] = v1[0];
    }
    else
    {
      if ( v0[0] == 0 )
      {
	tangent[0] = v1[1];
	tangent[1] = 0;
	tangent[2] = v1[0];
      }
      else
      {
	tangent[0] = v1[1] * v0[1];
	tangent[1] = v1[1] * v0[0];
	tangent[2] = v0[1] * v1[0];
      }
    }
  }
  else if ( mainAxis == Y )
  {
    if ( v0[1] == 0 || ( v1[1] == 0 && v0[0] == 0 ) )
    {
      tangent[0] = v0[1];
      tangent[1] = v1[1];
      tangent[2] = v1[0];
    }
    else
    {
      if ( v1[0] == 0 )
      {
	tangent[0] = v0[1];
	tangent[1] = v0[0];
	tangent[2] = 0;
      }
      else
      {
	tangent[0] = v1[1] * v0[1];
	tangent[1] = v1[1] * v0[0];
	tangent[2] = v0[0] * v1[0];
      }
    }
  }
  else
  {
    if ( v0[1] == 0 || ( v0[0] == 0 && v1[0] == 0 ) )
    {
      tangent[0] = v0[1];
      tangent[1] = v1[1];
      tangent[2] = v1[0];
    }
    else
    {
      if ( v1[1] == 0 )
      {
	tangent[0] = v0[1];
	tangent[1] = 0;
	tangent[2]= v0[0];
      }
      else
      {
	tangent[0] = v0[1] * v1[0];
	tangent[1] = v1[1] * v0[0];
	tangent[2] = v0[0] * v1[0];
      }
    }
  }
  return tangent;
}