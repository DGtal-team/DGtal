/**
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 **/

#pragma once

/**
 * @file OgreEventCatcher.h
 * @author Anis Benyoub (\c anis.benyoub@insa-lyon.fr )
 * Liris CNRS
 *
 * @date 2012/07/13
 *
 * Header file for module OgreEventCatcher.mm
 *
 * This file is part of the DGtal library.
 */

#if defined(OgreEventCatcher_RECURSES)
#error Recursive header files inclusion detected in OgreEventCatcher.h
#else // defined(OgreEventCatcher_RECURSES)
/** Prevents recursive inclusion of headers. */
#define OgreEventCatcher_RECURSES

#if !defined OgreEventCatcher_h
/** Prevents repeated inclusion of headers. */
#define OgreEventCatcher_h

#include "OgrePlatform.h"

#if OGRE_PLATFORM != OGRE_PLATFORM_APPLE
#error This header is for use with Mac OS X only
#endif

#ifdef __OBJC__

#import <Cocoa/Cocoa.h>
#include "MyApplication.h"
// All this does is suppress some messages in the run log.  NSApplication does not
// implement buttonPressed and apps without a NIB have no target for the action.
@implementation NSApplication (_suppressUnimplementedActionWarning)
- (void) buttonPressed:(id)sender
{
/* Do nothing */
}
@end


#if defined(MAC_OS_X_VERSION_10_6) && MAC_OS_X_VERSION_MAX_ALLOWED >= MAC_OS_X_VERSION_10_6
@interface OgreEventCatcher : NSObject <NSApplicationDelegate>
#else
@interface OgreEventCatcher : NSObject
#endif
{
  NSTimer *mTimer;
  NSDate *mDate;
  double mLastFrameTime;
  double mStartTime;
}

- (void)startRendering;
- (void)renderOneFrame:(id)sender;

@property (retain) NSTimer *mTimer;
@property (nonatomic) double mLastFrameTime;
@property (nonatomic) double mStartTime;

@end

#if __LP64__
static id myOgreEventCatcher;
#endif

@implementation OgreEventCatcher

@synthesize mTimer;
@dynamic mLastFrameTime;
@dynamic mStartTime;

- (double)mLastFrameTime
{
  return mLastFrameTime;
}

- (void)setLastFrameTime:(double)frameInterval
{
  // Frame interval defines how many display frames must pass between each time the
  // display link fires. The display link will only fire 30 times a second when the
  // frame internal is two on a display that refreshes 60 times a second. The default
  // frame interval setting of one will fire 60 times a second when the display refreshes
  // at 60 times a second. A frame interval setting of less than one results in undefined
  // behavior.
  if (frameInterval >= 1)
    {
      mLastFrameTime = frameInterval;
    }
}

- (void)startRendering {
    
  NSAutoreleasePool * pool = [[NSAutoreleasePool alloc] init];
  mLastFrameTime = 1;
  mStartTime = 0;
  mTimer = nil;
   
  mTimer = [NSTimer scheduledTimerWithTimeInterval:(NSTimeInterval)(1.0f / 60.0f) * mLastFrameTime
  	    target:self
  	    selector:@selector(renderOneFrame:)
  	    userInfo:nil
  	    repeats:YES];
  [pool release];
}
	    - (void)applicationDidFinishLaunching:(NSNotification *)application {
	      mLastFrameTime = 1;
	      mStartTime = 0;
	      mTimer = nil;
    
	      [self startRendering];
	    }



	    - (void)renderOneFrame:(id)sender
	    {
	      if(InputListener::getSingletonPtr()->viewerIsRunning() &&
		 Ogre::Root::getSingletonPtr() && Ogre::Root::getSingleton().isInitialised())
		{
		  mStartTime = InputListener::getSingletonPtr()->getTimer()->getMillisecondsCPU();
            
		  InputListener::getSingletonPtr()->getKeyBoard()->capture();
		  InputListener::getSingletonPtr()->getMouse()->capture();
            
		  InputListener::getSingletonPtr()->updateViewer(mLastFrameTime);
		  ViewerOgre3D::getSingleton().getOgreRoot()->renderOneFrame();
            
		  mLastFrameTime = InputListener::getSingletonPtr()->getTimer()->getMillisecondsCPU() - mStartTime;
		}
	      else
		{
		  [mTimer invalidate];
		  mTimer = nil;
		}
	    }

- (void)dealloc {
  if(mTimer)
    {
      [mTimer invalidate];
      mTimer = nil;
    }
    
  [super dealloc];
}

@end


#endif
#endif /* OgreEventCatcher_H_ */

#undef OgreEventCatcher_RECURSES
#endif // else defined(OgreEventCatcher_RECURSES)
