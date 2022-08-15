#import <Foundation/Foundation.h>

@interface ROS2Bridge : NSObject

- (void)startPublishing;
- (void)stopPublishing;

- (void)startListening;

@end
