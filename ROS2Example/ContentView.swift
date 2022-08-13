//
//  ContentView.swift
//  ROS2Example
//
//  Created by Lightech on 8/12/22.
//

import SwiftUI

let ros2 = ROS2Bridge()

struct ContentView: View {
    var body: some View {
        VStack {
            Button("Start publishing", action: {
                ros2.startPublishing()
            })
            Button("Start listening", action: {
                ros2.startListening()
            })
        }
    }
}

struct ContentView_Previews: PreviewProvider {
    static var previews: some View {
        ContentView()
    }
}
