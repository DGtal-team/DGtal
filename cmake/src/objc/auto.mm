#import <Foundation/Foundation.h>

int main (int argc, const char * argv[])
{
    NSAutoreleasePool * pool = [[NSAutoreleasePool alloc] init];

    // This is a basic program
    NSLog(@"Hello, World!");
    [pool drain];
    return 0;
}
