//
//  BLEManager.swift
//  Mobile Climate Lab
//
//  Created by Eslam Othma on 2024-02-13.
//

import Foundation
import CoreBluetooth
@Published var isConnected: Bool = false

class BLEManager {
    var onDataReceived: (() -> Void)?
}

class BLEManager: NSObject, ObservableObject, CBCentralManagerDelegate, CBPeripheralDelegate {
    var centralManager: CBCentralManager!
    var myPeripheral: CBPeripheral?
    @Published var temperatureMessage: String = ""
    
    override init() {
        super.init()
        centralManager = CBCentralManager(delegate: self, queue: nil)
    }
    
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        if central.state == .poweredOn {
            centralManager.scanForPeripherals(withServices: [CBUUID(string: "12345678-1234-1234-1234-123456789ABC")], options: nil)
        } else {
            print("Bluetooth is not available.")
        }
    }
    
    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String : Any], rssi RSSI: NSNumber) {
        centralManager.stopScan()
        myPeripheral = peripheral
        myPeripheral?.delegate = self
        centralManager.connect(peripheral, options: nil)
    }
    
    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        print("Connected to \(peripheral.name ?? "device")")
        DispatchQueue.main.async {
            self.isConnected = true
        }
        peripheral.discoverServices([CBUUID(string: "12345678-1234-1234-1234-123456789ABC")])
    }
    
    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        print("Disconnected from \(peripheral.name ?? "device")")
        DispatchQueue.main.async {
            self.isConnected = false
        }
        
        func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
            guard let services = peripheral.services else { return }
            for service in services {
                peripheral.discoverCharacteristics(nil, for: service)
            }
        }
        
        func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
            guard let characteristics = service.characteristics else { return }
            for characteristic in characteristics {
                if characteristic.uuid == CBUUID(string: "ABCD5678-1234-1234-1234-123456789ABC") {
                    peripheral.setNotifyValue(true, for: characteristic)
                }
            }
        }
        
        func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
            if let value = characteristic.value, let temperature = String(data: value, encoding: .utf8) {
                DispatchQueue.main.async {
                    self.temperatureMessage = "Temperature: \(temperature)"
                }
            }
        }
    }
}
