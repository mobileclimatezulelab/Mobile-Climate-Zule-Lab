//
//  Export.swift
//  Mobile Climate Lab
//
//  Created by Zean A.A. Ghanmeh on 2024-02-09.
//

import SwiftUI
import UniformTypeIdentifiers
import Foundation
import CoreLocation
import MapKit


struct Export: View {
    @State var First_isClicked = false
    @State var Second_isClicked = false
    @State var Third_isClicked = false
    @State var Fourth_isClicked = false
    @State var Fifth_isClicked = false
    @State var Sixth_isClicked = false
    @State var Chart_isClicked = false
    @State var Values_isClicked = false
    @State var Export_isClicked = true
    @State var Value_Page = false
    @State var Graph_Page = false
    @State var Export_Page = true
    @State var Number = 0.0
    @State var Unit = "--"
    @State var appeared: Double = 0.0
    @Environment(\.colorScheme) var colorScheme
    @Environment(\.presentationMode) var presentationMode: Binding<PresentationMode>
    @ObservedObject var bleManagerShared = BLEManager.shared
    @ObservedObject var sensorDataManager = SensorDataManager.shared
    @State private var showShareSheet = false
    @State private var fileURL: URL?
    @State private var region = MKCoordinateRegion()
    
    var body: some View {
        let isConnected = BLEManager.shared.isConnected
        let percentage = bleManagerShared.percentage

        if Export_Page{
            // Zstack showing all visual aspects of APP
            ZStack {
                //ZULE LAB Title Block
                Text("ZULE - \n" + "Mobile Climate Lab")
                    .font(.system(size: UIScreen.screenWidth/13.1, weight: .heavy, design: .rounded))
                //First part of .foregroundColor determines Color mode device
                    .foregroundColor(colorScheme == .dark ? Color(red : 0.5, green: 0.6, blue: 1.6) : Color(red : 0.99, green: 0.99, blue: 0.7))
                //Centers The text in the shape
                    .multilineTextAlignment(.center)
                    .position(x: UIScreen.screenWidth/2, y: 3*UIScreen.screenHeight/100)
                Text("\(percentage)%" )
                    .font(.system(size: UIScreen.screenWidth/30, weight: .heavy, design: .rounded))
                    .position(x: UIScreen.screenWidth/1.08, y: UIScreen.screenWidth/15)
                
                Circle()
                           .fill(isConnected ? Color.green : Color.red)
                           .frame(width: 20, height: 20)
                           .position(x: UIScreen.screenWidth - 30, y: 50) // Adjust position as needed
                           .onAppear {
                           }
                Map(coordinateRegion: $region, showsUserLocation: true, userTrackingMode: .constant(.follow))
                           .onAppear {
                               if let currentLocation = sensorDataManager.currentLocation {
                                   region = MKCoordinateRegion(center: currentLocation.coordinate, latitudinalMeters: 500, longitudinalMeters: 500)
                               }
                           }
                           .frame(width: UIScreen.screenWidth, height: 1.25*UIScreen.screenHeight/2)
                           .position(x: UIScreen.screenWidth/2, y: UIScreen.screenHeight/2.2 )

                           .onAppear{
                               CLLocationManager().requestWhenInUseAuthorization()
                               CLLocationManager().requestAlwaysAuthorization()
                           }
                Button(action: {
                    exportAndShareCSV()
                }) {
                    Image(systemName: "square.and.arrow.up")
                        .font(.title) // Adjust the size as needed
                        .foregroundColor(.blue) // Adjust the color as needed
                }
                .sheet(isPresented: $showShareSheet) {
                    if let url = self.fileURL {
                        ActivityView(activityItems: [url], applicationActivities: nil)
                    } else {
                        EmptyView()
                    }
                }
                .position(x: UIScreen.screenWidth/2, y: UIScreen.screenHeight/1.25)
//                Button("Export and Share CSV") {
//                    exportAndShareCSV()
//                }.sheet(isPresented: $showShareSheet) {
//                    if let url = self.fileURL {
//                        ActivityView(activityItems: [url], applicationActivities: nil)
//                    } else {
//                        EmptyView()
//                    }
//                }
//                .position(x: UIScreen.screenWidth/2, y: UIScreen.screenHeight/1.25)

                //Naviagtion Bar Display
                Rectangle()
                    .fill(colorScheme == .light ? Color(red : 0.8, green: 0.8, blue: 0.8) : Color(red : 0.1, green: 0.1, blue: 0.1))
                    .frame(width: UIScreen.screenWidth, height: 3*UIScreen.screenHeight/11)
                    .position(x: UIScreen.screenWidth/2, y: 99.5*UIScreen.screenHeight/100)
                
                /**************************************************************************************************************************************************************************************************************************************************************/
                //Naviagtion Bar Value Display
                Button(action: {
                    //Take to the chart page
                    self.Values_isClicked = false
                    self.Export_isClicked = false
                    self.Export_Page = false
                    self.Value_Page = false
                }){
                    //Naviagtion Bar Value Active Use check
                    if self.Chart_isClicked{
                        Image(systemName: "chart.xyaxis.line")
                            .resizable()
                            .frame(width: UIScreen.screenWidth/13.1, height: UIScreen.screenWidth/13.1)
                            .foregroundStyle(.white)
                            .position(x: UIScreen.screenWidth/2, y: 89*UIScreen.screenHeight/100)
                            .shadow(color: colorScheme == .light ? .black : .white, radius: UIScreen.screenWidth/39.3)
                    }
                    else {
                        Image(systemName: "chart.xyaxis.line")
                            .resizable()
                            .frame(width: UIScreen.screenWidth/15.72, height: UIScreen.screenWidth/15.72)
                            .foregroundStyle(.white)
                            .position(x: UIScreen.screenWidth/2, y: 89*UIScreen.screenHeight/100)
                    }
                }
                //Gesture detection for Naviagtion Bar Value pressing or holding
                .simultaneousGesture(
                    LongPressGesture(minimumDuration: 0.01).onEnded({ _ in
                        self.Chart_isClicked = true
                        self.Graph_Page = true
                    })
                )
                //Style of Naviagtion Bar Value button
                .buttonStyle(PlainButtonStyle())
                
                /**************************************************************************************************************************************************************************************************************************************************************/
                
                //Naviagtion Bar Graph Display
                Button(action: {
                    //Take to the Value page
                    self.Chart_isClicked = false
                    self.Export_isClicked = false
                    self.Graph_Page = false
                    self.Export_Page = false
                }){
                    if self.Values_isClicked{
                        Image(systemName: "thermometer.sun.fill")
                            .resizable()
                            .frame(width: UIScreen.screenWidth/13.1, height: UIScreen.screenWidth/13.1)
                            .foregroundStyle(.white)
                            .position(x: UIScreen.screenWidth/5, y: 89*UIScreen.screenHeight/100)
                            .shadow(color: colorScheme == .light ? .black : .white, radius: UIScreen.screenWidth/39.3)
                    }
                    else {
                        Image(systemName: "thermometer.sun")
                            .resizable()
                            .frame(width: UIScreen.screenWidth/15.72, height: UIScreen.screenWidth/15.72)
                            .foregroundStyle(.white)
                            .position(x: UIScreen.screenWidth/5, y: 89*UIScreen.screenHeight/100)
                    }
                }
                //Gesture detection for Naviagtion Bar Graph pressing or holding
                .simultaneousGesture(
                    LongPressGesture(minimumDuration: 0.01).onEnded({ _ in
                        self.Values_isClicked = true
                        self.Value_Page = true
                    })
                )
                //Style of Naviagtion Bar Graph button
                .buttonStyle(PlainButtonStyle())
                
                /**************************************************************************************************************************************************************************************************************************************************************/
                
                //Naviagtion Bar Exporting Display
                Button(action: {
                    //Take to the Export page
                    self.Chart_isClicked = false
                    self.Values_isClicked = false
                    self.Value_Page = false
                    self.Graph_Page = false
                }){
                    //Naviagtion Bar Export Active Use check
                    if self.Export_isClicked{
                        Image(systemName: "map.fill")
                            .resizable()
                            .frame(width: UIScreen.screenWidth/13.1, height: 35)
                            .foregroundStyle(.white)
                            .position(x: 4*UIScreen.screenWidth/5, y: 89*UIScreen.screenHeight/100)
                            .shadow(color: colorScheme == .light ? .black : .white, radius: UIScreen.screenWidth/39.3)
                    }
                    else {
                        Image(systemName: "map")
                            .resizable()
                            .frame(width: UIScreen.screenWidth/15.72, height: UIScreen.screenWidth/13.1)
                            .foregroundStyle(.white)
                            .position(x: 4*UIScreen.screenWidth/5, y: 89*UIScreen.screenHeight/100)
                    }
                }
                //Gesture detection for Naviagtion Bar Export pressing or holding
                .simultaneousGesture(
                    LongPressGesture(minimumDuration: 0.01).onEnded({ _ in
                        self.Export_isClicked = true
                        self.Export_Page = true
                    })
                )
                //Style of Naviagtion Bar Export button
                .buttonStyle(PlainButtonStyle())
            } //End of ZStack
            
            /**************************************************************************************************************************************************************************************************************************************************************/
            
            //Background APP Colour
            .background(colorScheme == .light ? LinearGradient(stops: [
                Gradient.Stop(color: Color(red : 0.99, green: 0.56, blue: 0.11), location: 0.45),
                Gradient.Stop(color: .white, location: 0.6)
            ], startPoint: .top, endPoint: .bottom).edgesIgnoringSafeArea(.all):
                            LinearGradient(stops: [
                                Gradient.Stop(color: Color(red : 0.2, green: 0.11, blue: 0.7),
                                              location: 0.45),
                                Gradient.Stop(color: .black, location: 0.6)
                            ], startPoint: .top, endPoint: .bottom).edgesIgnoringSafeArea(.all))
        }
        else if Graph_Page{
            Graphs()
            }
        else if Value_Page{
            ContentView()
        }
    }
    private func exportAndShareCSV() {
           let csvString = sensorDataManager.sensorDataToCSV()
        
           let filename = sensorDataManager.fileName
           let path = FileManager.default.temporaryDirectory.appendingPathComponent(filename)

           do {
               try csvString.write(to: path, atomically: true, encoding: .utf8)
               self.fileURL = path
               self.showShareSheet = true
           } catch {
               print("Failed to write CSV file: \(error)")
           }
       }

       private func shareSheet() -> some View {
           // Make sure to conditionally show the share sheet only if the file URL is available
           if let url = fileURL {
               return AnyView(ActivityView(activityItems: [url], applicationActivities: nil))
           } else {
               return AnyView(EmptyView())
           }
       }
   }


struct ActivityView: UIViewControllerRepresentable {
    var activityItems: [Any]
    var applicationActivities: [UIActivity]? = nil

    func makeUIViewController(context: Context) -> UIActivityViewController {
        return UIActivityViewController(activityItems: activityItems, applicationActivities: applicationActivities)
    }

    func updateUIViewController(_ uiViewController: UIActivityViewController, context: Context) {}
}



#Preview {
    Export()
}
