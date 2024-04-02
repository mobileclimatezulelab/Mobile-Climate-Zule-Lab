//
//  Mobile_Climate_LabApp.swift
//  Mobile Climate Lab
//
//  Created by Zean A.A. Ghanmeh on 2024-01-11.
//

import SwiftUI

@main
struct Mobile_Climate_LabApp: App {
    var bleManager = BLEManager.shared
    var sensorDataManager = SensorDataManager.shared

    var body: some Scene {
        WindowGroup {
            ContentView()
        }
    }
}
