#ifndef __MyApplicationMain_H__
#define __MyApplicationMain_H__

#include "OgrePlatform.h"

#ifdef __OBJC__
#import <Cocoa/Cocoa.h>

int MyApplicationMain(int argc, const char **argv);

@interface MyApplication : NSApplication
{
	bool shouldKeepRunning;
}

- (void)run;
- (void)terminate:(id)sender;

@end

#include "MyApplication.ih"
#endif
#endif
