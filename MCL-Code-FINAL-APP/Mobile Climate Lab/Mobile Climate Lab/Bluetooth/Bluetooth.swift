//
//  Bluetooth.swift
//  Mobile Climate Lab
//
//  Created by Eslam Othma on 2024-02-21.
//
import CoreBluetooth
import Combine


 class BLEManager: NSObject, ObservableObject, CBCentralManagerDelegate, CBPeripheralDelegate {
     
    static let shared = BLEManager()
    static let percentage = BLEManager.shared.percentage
    
    @Published var isConnected: Bool = false
    @Published var receivedData: String = ""
    @Published var solarTemperature: Double = 0.0
    @Published var ambientTemperature: Double = 0.0
    @Published var ambientHumidity: Double = 0.0
    @Published var pm25Env: Double = 0.0
    @Published var percentage: Int = 0
    
    let customServiceUUID = CBUUID(string: "12345678-1234-1234-1234-123456789ABC")
    let commandCharacteristicUUID = CBUUID(string: "ABCD1234-1234-1234-1234-123456789ABC")
    let responseCharacteristicUUID = CBUUID(string: "ABCD5678-1234-1234-1234-123456789ABC")
    
    // Initializations.
    var centralManager: CBCentralManager!
    var discoveredPeripheral: CBPeripheral?
    var commandCharacteristic: CBCharacteristic?
    var responseCharacteristic: CBCharacteristic?
    
    override init() {
        super.init()
        centralManager = CBCentralManager(delegate: self, queue: nil) // Consider using a background queue for better performance.
    }
    
    
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        switch central.state {
        case .poweredOn:
            print("Bluetooth is powered on. Starting scan...")
            centralManager.scanForPeripherals(withServices: [customServiceUUID], options: nil)
        case .poweredOff:
            print("Bluetooth is powered off.")
            isConnected = false
        default:
            print("Bluetooth is in an unsupported, unauthorized, or unknown state.")
            isConnected = false
        }
    }
    
    
    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String : Any], rssi RSSI: NSNumber) {
        print("Discovered peripheral: \(peripheral.name ?? "Unnamed")")
        discoveredPeripheral = peripheral
        centralManager.stopScan()
        centralManager.connect(peripheral, options: nil)
    }
    
    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        print("Connected to peripheral: \(peripheral)")
        self.isConnected = true
        peripheral.delegate = self
        peripheral.discoverServices([customServiceUUID])
        isConnected = true
    }
    
    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        print("Disconnected from peripheral")
        self.isConnected = false
        if central.state == .poweredOn {
            print("Starting scan...")
            centralManager.scanForPeripherals(withServices: [customServiceUUID], options: nil)
            // make it zero
        } else {
            print("Bluetooth is not powered on. Cannot start scan.")
        }
    }
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        if let error = error {
            print("Error discovering services: \(error.localizedDescription)")
            return
        }
        
        guard let services = peripheral.services else { return }
        for service in services where service.uuid == customServiceUUID {
            peripheral.discoverCharacteristics([commandCharacteristicUUID, responseCharacteristicUUID], for: service)
        }
    }
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        if let error = error {
            print("Error discovering characteristics: \(error.localizedDescription)")
            return
        }
        
        guard let characteristics = service.characteristics else { return }
        for characteristic in characteristics {
            if characteristic.uuid == commandCharacteristicUUID {
                commandCharacteristic = characteristic
            } else if characteristic.uuid == responseCharacteristicUUID {
                responseCharacteristic = characteristic
                peripheral.setNotifyValue(true, for: characteristic)
            }
        }
    }
    
    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
        if let error = error {
            print("Error receiving characteristic update: \(error.localizedDescription)")
            return
        }
        
        if characteristic.uuid == responseCharacteristicUUID, let value = characteristic.value, let string = String(data: value, encoding:.utf8) {
            DispatchQueue.main.async {
                self.receivedData = string
                let sensorValues = string.split(separator: ",").map { String($0) }
                
                if sensorValues.count >= 5,
                   let solarTemp = Double(sensorValues[0]),
                   let ambientTemp = Double(sensorValues[1]),
                   let humidity = Double(sensorValues[2]),
                   let pm25 = Double(sensorValues[3]),
                   let percentage = Double(sensorValues[4]) {
                    
                    self.solarTemperature = solarTemp
                    self.ambientTemperature = ambientTemp
                    self.ambientHumidity = humidity
                    self.pm25Env = pm25
                    self.percentage = Int(percentage)
                    
                    if let solarTemp = Double(sensorValues[0]), let ambientTemp = Double(sensorValues[1]), let humidity = Double(sensorValues[2]), let pm25 = Double(sensorValues[3]) {
                        
                        SensorDataManager.addSensorData(solarTemp: solarTemp, ambientTemp: ambientTemp, humidity: humidity, pm25: pm25)
                    }
                } else {
                    print("Error parsing sensor values")
                }
            }
        }
    }
}
