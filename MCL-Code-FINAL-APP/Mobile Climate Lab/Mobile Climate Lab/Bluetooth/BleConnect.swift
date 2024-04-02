//
//  ContentViewtest.swift
//  Mobile Climate Lab
//
//  Created by Eslam Othma on 2024-02-28.
//

import SwiftUI


struct BleConnect: View {
    @EnvironmentObject var bleManager: BLEManager

    var body: some View {
        VStack {
            // Display the connection status
            Text(bleManager.isConnected ? "Connected" : "Disconnected")
                .font(.title)
                .foregroundColor(bleManager.isConnected ? .green : .red)
                .padding()

            // A circle indicator that changes color based on the connection status
            Circle()
                .fill(bleManager.isConnected ? Color.green : Color.red)
                .frame(width: 50, height: 50)
                .padding(.bottom, 20)


            }
        }
    }
