//
//  SensorDataManager.swift
//  Mobile Climate Lab
//
//  Created by Eslam Othma on 2024-03-15.
//

import Foundation
import Combine
import CoreLocation

// SensorData remains unchanged
struct SensorData {
    var timestamp: Date
    var solarTemperature: Double
    var ambientTemperature: Double
    var ambientHumidity: Double
    var pm25Env: Double
    var latitude: Double
    var longitude: Double
}

// Simplified SensorDataManager for real-time graphing
class SensorDataManager: ObservableObject {
    static let shared = SensorDataManager()
    static var hasData = false;
    
    var currentLocation: CLLocation?
    
    @Published var sensorDataArray: [SensorData]

    
    let fileName = "SensorData.csv"
    
    private lazy var dateFormatter: DateFormatter = {
    let formatter = DateFormatter()
    formatter.dateFormat = "yyyy-MM-dd HH:mm:ss"
    return formatter
    }()
    
    //Initialize data array to empty after init.
    private init() {
        self.sensorDataArray = [];
    }
    
    // Static method to interact with the dynamic array
    static func addData(value: SensorData) {
        if(SensorDataManager.hasData == false){
            SensorDataManager.hasData = true
        }
        
        DispatchQueue.main.async {
            SensorDataManager.shared.sensorDataArray.append(value)
            // If the array gets too large, consider removing old entries
            if SensorDataManager.shared.sensorDataArray.count > 1000 { // Example limit
                SensorDataManager.shared.sensorDataArray.removeFirst()
            }
        }
    }

    // Static method to get all values
    static func getData() -> [SensorData] {
        return SensorDataManager.shared.sensorDataArray
    }
    
    //var nullSensorData = SensorData(timestamp: Date(), solarTemperature: 0.0, ambientTemperature: 0.0, ambientHumidity: 0.0, pm25Env: 0.0)
//    static func getLastSensorData() -> SensorData? {
//        return SensorDataManager.shared.sensorDataArray.last;
//    }
    
    static func getLastSensorData() -> SensorData {
        return SensorDataManager.shared.sensorDataArray.last ?? SensorData(
            timestamp: Date(),
            solarTemperature: 0.0,
            ambientTemperature: 0.0,
            ambientHumidity: 0.0,
            pm25Env: 0.0,
            latitude: 0.0, // Default latitude value
            longitude: 0.0 // Default longitude value
        )
    }

    static func addSensorData(solarTemp: Double, ambientTemp: Double, humidity: Double, pm25: Double, timestamp: Date = Date()) {
        // Use the current location if available, otherwise default to 0.0
        let latitude = SensorDataManager.shared.currentLocation?.coordinate.latitude ?? 0.0
        let longitude = SensorDataManager.shared.currentLocation?.coordinate.longitude ?? 0.0
        
        let newData = SensorData(timestamp: timestamp, solarTemperature: solarTemp, ambientTemperature: ambientTemp, ambientHumidity: humidity, pm25Env: pm25, latitude: latitude, longitude: longitude)
        
        addData(value: newData)
    }
    // Convert sensor data to CSV format
    func sensorDataToCSV() -> String {
        let header = "Timestamp,Solar Temperature,Ambient Temperature,Humidity,PM2.5,Latitude,Longitude"
        let csvRows = sensorDataArray.map { entry in
            "\(dateFormatter.string(from: entry.timestamp)),\(entry.solarTemperature),\(entry.ambientTemperature),\(entry.ambientHumidity),\(entry.pm25Env),\(entry.latitude),\(entry.longitude)"
        }
        
        return ([header] + csvRows).joined(separator: "\n")
    }
}
