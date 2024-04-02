//
//  ContentView.swift
//  Mobile Climate Lab
//
//  Created by Zean A.A. Ghanmeh on 2024-01-11.
//

import SwiftUI
import Combine
import MapKit
import CoreLocation


extension UIScreen{
   static let screenWidth = UIScreen.main.bounds.size.width
   static let screenHeight = UIScreen.main.bounds.size.height
   static let screenSize = UIScreen.main.bounds.size
}

struct GrowingAnimButton: ButtonStyle {
    @Environment(\.colorScheme) var colorScheme
    func makeBody(configuration: Configuration) -> some View {
        configuration.label
            .background(colorScheme == .light ? Color(red : 0.2, green: 0.11, blue: 0.7) : Color(red : 0.5, green: 0.44, blue: 1.6))
            .foregroundColor(.white)
            .clipShape(Capsule())
            .position(x: UIScreen.screenWidth/4, y: 7*UIScreen.screenHeight/16)
            .scaleEffect(configuration.isPressed ? 1.2 : 1)
            .animation(.easeOut(duration: 0.2), value: configuration.isPressed)
    }
}

struct ContentView: View {
    @State var First_isClicked = false
    @State var Second_isClicked = false
    @State var Third_isClicked = false
    @State var Fourth_isClicked = false
    @State var Fifth_isClicked = false
    @State var Sixth_isClicked = false
    @State var Chart_isClicked = false
    @State var Values_isClicked = true
    @State var Export_isClicked = false
    @State var Number = 0.0
    @State var Unit = "--"
    @State var appeared: Double = 0.0
    @State var Value_Page = true
    @State var Graph_Page = false
    @State var Export_Page = false
    
    @Environment(\.colorScheme) var colorScheme
    @Environment(\.presentationMode) var presentationMode: Binding<PresentationMode>
    
    
    
    @ObservedObject var bleManagerShared = BLEManager.shared
    @ObservedObject var sensorDataManager = SensorDataManager.shared
   

    var body: some View {
        // Zstack showing all visual aspects of APP
        let isConnected = BLEManager.shared.isConnected
        //var isConnected = SensorDataManager.hasData
        let bleManager = SensorDataManager.getLastSensorData()
        let percentage = bleManagerShared.percentage
        if Value_Page{
            ZStack {
                //ZULE LAB Title Block
                Text("ZULE - \n" + "Mobile Climate Lab")
                    .font(.system(size: UIScreen.screenWidth/13.1, weight: .heavy, design: .rounded))
                //First part of .foregroundColor determines Color mode device
                    .foregroundColor(colorScheme == .dark ? Color(red : 0.5, green: 0.6, blue: 1.6) : Color(red : 0.99, green: 0.99, blue: 0.7))
                //Centers The text in the shape
                    .multilineTextAlignment(.center)
                    .position(x: UIScreen.screenWidth/2, y: 3*UIScreen.screenHeight/100)
                
                /**************************************************************************************************************************************************************************************************************************************************************/
                //red : 0.99, green: 0.56, blue: 0.11
                    .onAppear{
                        CLLocationManager().requestWhenInUseAuthorization()
                        CLLocationManager().requestAlwaysAuthorization()
                    }
                //Main Shape Display
                Circle()
                    .fill(isConnected ? Color.green : Color.red)
                    .frame(width: UIScreen.screenWidth/19.65, height: UIScreen.screenWidth/19.65)
                    .position(x: UIScreen.screenWidth/1.08, y: UIScreen.screenWidth/7.86) // Adjust position as needed
                    .onAppear {}
                Text("\(percentage)%" )
                    .font(.system(size: UIScreen.screenWidth/30, weight: .heavy, design: .rounded))
                    .position(x: UIScreen.screenWidth/1.08, y: UIScreen.screenWidth/15)
                Circle()
                //First part of .fill determines Color mode device
                    .fill(colorScheme == .light ? Color(red : 0.99, green: 0.8, blue: 0.11) : Color(red : 0.5, green: 0.11, blue: 1.6))
                    .frame(width: UIScreen.screenWidth/1.572)
                    .position(x: UIScreen.screenWidth/2, y: 22*UIScreen.screenHeight/100)
                //Main Value Display
                if First_isClicked {
                    Text(bleManager.ambientTemperature.formatted(.number.precision(.fractionLength(2))))
                        .foregroundStyle(.white)
                        .position(x: UIScreen.screenWidth/2, y: UIScreen.screenHeight/5)
                        .font(.system(size: UIScreen.screenWidth/10.92, weight: .heavy, design: .rounded))
                        .background(Circle()
                                    //First part of .fill determines Color mode device
                            .fill(colorScheme == .light ? Color(red : 0.99, green: 0.56, blue: 0.11) : Color(red : 0.2, green: 0.11, blue: 0.7))
                            .frame(width: UIScreen.screenWidth/1.64)
                            .position(x: UIScreen.screenWidth/2, y: 22*UIScreen.screenHeight/100))
                }
                else if Second_isClicked {
                    Text(bleManager.ambientHumidity.formatted(.number.precision(.fractionLength(2))))
                        .foregroundStyle(.white)
                        .position(x: UIScreen.screenWidth/2, y: UIScreen.screenHeight/5)
                        .font(.system(size: UIScreen.screenWidth/10.92, weight: .heavy, design: .rounded))
                        .background(Circle()
                                    //First part of .fill determines Color mode device
                            .fill(colorScheme == .light ? Color(red : 0.99, green: 0.56, blue: 0.11) : Color(red : 0.2, green: 0.11, blue: 0.7))
                            .frame(width: UIScreen.screenWidth/1.64)
                            .position(x: UIScreen.screenWidth/2, y: 22*UIScreen.screenHeight/100))
                }
                else if Third_isClicked {
                    Text(bleManager.solarTemperature.formatted(.number.precision(.fractionLength(2))))
                        .foregroundStyle(.white)
                        .position(x: UIScreen.screenWidth/2, y: UIScreen.screenHeight/5)
                        .font(.system(size: UIScreen.screenWidth/10.92, weight: .heavy, design: .rounded))
                        .background(Circle()
                                    //First part of .fill determines Color mode device
                            .fill(colorScheme == .light ? Color(red : 0.99, green: 0.56, blue: 0.11) : Color(red : 0.2, green: 0.11, blue: 0.7))
                            .frame(width: UIScreen.screenWidth/1.64)
                            .position(x: UIScreen.screenWidth/2, y: 22*UIScreen.screenHeight/100))
                }
                else if Sixth_isClicked {
                    Text(bleManager.pm25Env.formatted(.number.precision(.fractionLength(2))))
                        .foregroundStyle(.white)
                        .position(x: UIScreen.screenWidth/2, y: UIScreen.screenHeight/5)
                        .font(.system(size: UIScreen.screenWidth/10.92, weight: .heavy, design: .rounded))
                        .background(Circle()
                                    //First part of .fill determines Color mode device
                            .fill(colorScheme == .light ? Color(red : 0.99, green: 0.56, blue: 0.11) : Color(red : 0.2, green: 0.11, blue: 0.7))
                            .frame(width: UIScreen.screenWidth/1.64)
                            .position(x: UIScreen.screenWidth/2, y: 22*UIScreen.screenHeight/100))
                }
                else{
                    Text(0.formatted(.number.precision(.fractionLength(2))))
                        .foregroundStyle(.white)
                        .position(x: UIScreen.screenWidth/2, y: UIScreen.screenHeight/5)
                        .font(.system(size: UIScreen.screenWidth/10.92, weight: .heavy, design: .rounded))
                        .background(Circle()
                                    //First part of .fill determines Color mode device
                            .fill(colorScheme == .light ? Color(red : 0.99, green: 0.56, blue: 0.11) : Color(red : 0.2, green: 0.11, blue: 0.7))
                            .frame(width: UIScreen.screenWidth/1.64)
                            .position(x: UIScreen.screenWidth/2, y: 22*UIScreen.screenHeight/100))
                }
                //Main Unit Display
                Text(Unit)
                    .foregroundStyle(.white)
                    .position(x: UIScreen.screenWidth/2, y: 3*UIScreen.screenHeight/10)
                    .font(.system(size: UIScreen.screenWidth/15.72, weight: .heavy, design: .rounded))
                
                /**************************************************************************************************************************************************************************************************************************************************************/
                
                //Ambient °C Display Button
                Capsule()
                    .fill(colorScheme == .light ? Color(red : 0.99, green: 0.8, blue: 0.11) : Color(red : 0.5, green: 0.44, blue: 1.6))
                    .frame(width: UIScreen.screenWidth/2.62, height: UIScreen.screenWidth/3.93)
                    .position(x: UIScreen.screenWidth/4, y: 7*UIScreen.screenHeight/16)
                //Ambient °C Animation Button
                Button(action: {
                    self.Second_isClicked = false
                    self.Third_isClicked = false
                    self.Fourth_isClicked = false
                    self.Fifth_isClicked = false
                    self.Sixth_isClicked = false
                    Number = bleManager.ambientTemperature
                    Unit = "Amb °C"
                }){
                    //Ambient °C Active Use check
                    if self.First_isClicked {
                        Text("\(bleManager.ambientTemperature.formatted(.number.precision(.fractionLength(2))))")
                            .foregroundStyle(.white)
                            .position(x: UIScreen.screenWidth/4, y: 7*UIScreen.screenHeight/16)
                            .font(.system(size: UIScreen.screenWidth/19.65, weight: .heavy, design: .rounded))
                        //Ambient °C Inner Display Button
                            .background(Capsule()
                                        //First part of .fill determines light mode device
                                .fill(colorScheme == .light ? Color(red : 0.99, green: 0.45, blue: 0.11) : Color(red : 0.5, green: 0.44, blue: 1.6))
                                .frame(width: UIScreen.screenWidth/2.81, height: UIScreen.screenWidth/4.37)
                                .shadow(color: colorScheme == .light ? .black : .white, radius: UIScreen.screenWidth/78.6)
                                .position(x: UIScreen.screenWidth/4, y: 7*UIScreen.screenHeight/16))}
                    else {
                        Text("\(bleManager.ambientTemperature.formatted(.number.precision(.fractionLength(2))))")                            .foregroundStyle(.white)
                            .position(x: UIScreen.screenWidth/4, y: 7*UIScreen.screenHeight/16)
                            .font(.system(size: UIScreen.screenWidth/19.65, weight: .heavy, design: .rounded))
                        //Ambient °C Inner Display Button
                            .background(Capsule()
                                .fill(colorScheme == .dark ? Color(red : 0.2, green: 0.11, blue: 0.7) : Color(red : 0.99, green: 0.56, blue: 0.11))
                                .frame(width: UIScreen.screenWidth/2.81, height: UIScreen.screenWidth/4.37)
                                .position(x: UIScreen.screenWidth/4, y: 7*UIScreen.screenHeight/16))}
                    
                }
                //Gesture detection for Ambient °C pressing or holding
                .simultaneousGesture(
                    LongPressGesture(minimumDuration: 0.01).onEnded({ _ in
                        self.First_isClicked = true
                    })
                )
                //Style of Ambient °C button
                .buttonStyle(PlainButtonStyle())
                //Ambient °C Button Displayed Text
                Text("Amb. °C")
                    .foregroundStyle(.white)
                    .position(x: UIScreen.screenWidth/4, y: 15*UIScreen.screenHeight/32)
                    .font(.system(size: UIScreen.screenWidth/19.65, weight: .heavy, design: .rounded))
                /**************************************************************************************************************************************************************************************************************************************************************/
                
                //Humidity % Display Button
                Capsule()
                    .fill(colorScheme == .light ? Color(red : 0.99, green: 0.8, blue: 0.11) : Color(red : 0.5, green: 0.44, blue: 1.6))
                    .frame(width: UIScreen.screenWidth/2.62, height: UIScreen.screenWidth/3.93)
                    .position(x: 3*UIScreen.screenWidth/4, y: 7*UIScreen.screenHeight/16)
                //Humidity % Animation Button
                Button(action: {
                    self.First_isClicked = false
                    self.Third_isClicked = false
                    self.Fourth_isClicked = false
                    self.Fifth_isClicked = false
                    self.Sixth_isClicked = false
                    Number = bleManager.ambientHumidity
                    Unit = "Hum. %"
                }){
                    //Humidity % Active Use check
                    if self.Second_isClicked {
                        Text("\(bleManager.ambientHumidity, specifier: "%.2f")")
                            .foregroundStyle(.white)
                            .position(x: 3*UIScreen.screenWidth/4, y: 7*UIScreen.screenHeight/16)
                            .font(.system(size: UIScreen.screenWidth/19.65, weight: .heavy, design: .rounded))
                        //Humidity % Inner Display Button
                            .background(Capsule()
                                .fill(colorScheme == .light ? Color(red : 0.99, green: 0.45, blue: 0.11) : Color(red : 0.5, green: 0.44, blue: 1.6))
                                .frame(width: UIScreen.screenWidth/2.81, height: UIScreen.screenWidth/4.37)
                                .shadow(color: colorScheme == .light ? .black : .white, radius: UIScreen.screenWidth/78.6)
                                .position(x: 3*UIScreen.screenWidth/4, y: 7*UIScreen.screenHeight/16))}
                    else {
                        Text("\(bleManager.ambientHumidity, specifier: "%.2f")")
                            .foregroundStyle(.white)
                            .position(x: 3*UIScreen.screenWidth/4, y: 7*UIScreen.screenHeight/16)
                            .font(.system(size: UIScreen.screenWidth/19.65, weight: .heavy, design: .rounded))
                        //Humidity % Inner Display Button
                            .background(Capsule()
                                .fill(colorScheme == .dark ? Color(red : 0.2, green: 0.11, blue: 0.7) : Color(red : 0.99, green: 0.56, blue: 0.11))
                                .frame(width: UIScreen.screenWidth/2.81, height: UIScreen.screenWidth/4.37)
                                .position(x: 3*UIScreen.screenWidth/4, y: 7*UIScreen.screenHeight/16))}
                    
                }
                //Gesture detection for Humidity % pressing or holding
                .simultaneousGesture(
                    LongPressGesture(minimumDuration: 0.01).onEnded({ _ in
                        self.Second_isClicked = true
                    })
                )
                //Style of Humidity % button
                .buttonStyle(PlainButtonStyle())
                //Humidity % Button Displayed Text
                Text("Humid. %")
                    .foregroundStyle(.white)
                    .position(x: 3*UIScreen.screenWidth/4, y: 15*UIScreen.screenHeight/32)
                    .font(.system(size: UIScreen.screenWidth/19.65, weight: .heavy, design: .rounded))
                
                /**************************************************************************************************************************************************************************************************************************************************************/
                
                //Solar Radiation °C  Animation Button
                Button(action: {
                    self.First_isClicked = false
                    self.Second_isClicked = false
                    self.Fourth_isClicked = false
                    self.Fifth_isClicked = false
                    self.Sixth_isClicked = false
                    Number = bleManager.solarTemperature
                    Unit = "Sol. °C"
                }){
                    //Solar Radiation °C Active Use check
                    if self.Third_isClicked {Text(bleManager.solarTemperature.formatted(.number.precision(.fractionLength(2))))
                            .foregroundStyle(.white)
                            .position(x: 2*UIScreen.screenWidth/4, y: 3*UIScreen.screenHeight/5)
                            .font(.system(size: 2*UIScreen.screenWidth/19.65, weight: .light, design: .serif))
                        //Solar Radiation °C Display Button
                            .background(RoundedRectangle(cornerRadius: UIScreen.screenWidth/15.72)
                                .fill(colorScheme == .light ? LinearGradient(stops: [
                                    Gradient.Stop(color: Color(red : 0.99, green: 0.56, blue: 0.11), location: 0),
                                    Gradient.Stop(color: .white, location: 0.9)
                                ], startPoint: .bottom, endPoint: .top):
                                        LinearGradient(stops: [
                                            Gradient.Stop(color: Color(red : 0.2, green: 0.11, blue: 0.7),
                                                          location: 0),
                                            Gradient.Stop(color: .black, location: 0.9)
                                        ], startPoint: .bottom, endPoint: .top))
                                    .frame(width: 2.75*UIScreen.screenWidth/3.144, height: UIScreen.screenWidth/3.144)
                                    .position(x: 2*UIScreen.screenWidth/4, y: 3*UIScreen.screenHeight/5)
                                    .shadow(color: colorScheme == .light ? .black : .white, radius: UIScreen.screenWidth/15.72))}
                    else {Text(bleManager.solarTemperature.formatted(.number.precision(.fractionLength(2))))
                            .foregroundStyle(.white)
                            .position(x: 2*UIScreen.screenWidth/4, y: 3*UIScreen.screenHeight/5)
                            .font(.system(size: 2*UIScreen.screenWidth/19.65, weight: .light, design: .serif))
                        //Solar Radiation °C Display Button
                            .background(RoundedRectangle(cornerRadius: UIScreen.screenWidth/15.72)
                                .fill(colorScheme == .light ? LinearGradient(stops: [
                                    Gradient.Stop(color: Color(red : 0.99, green: 0.56, blue: 0.11), location: 0),
                                    Gradient.Stop(color: .white, location: 0.9)
                                ], startPoint: .bottom, endPoint: .top):
                                        LinearGradient(stops: [
                                            Gradient.Stop(color: Color(red : 0.2, green: 0.11, blue: 0.7),
                                                          location: 0),
                                            Gradient.Stop(color: .black, location: 0.9)
                                        ], startPoint: .bottom, endPoint: .top))
                                    .frame(width: 2.75*UIScreen.screenWidth/3.144, height: UIScreen.screenWidth/3.144)
                                    .position(x: 2*UIScreen.screenWidth/4, y: 3*UIScreen.screenHeight/5))}
                    
                }
                //Gesture detection for Solar Radiation °C pressing or holding
                .simultaneousGesture(
                    LongPressGesture(minimumDuration: 0.01).onEnded({ _ in
                        self.Third_isClicked = true
                    })
                )
                //Style of Solar Radiation °C button
                .buttonStyle(PlainButtonStyle())
                //Solar Radiation °C Button Displayed Text
                Text("Solar °C")
                    .foregroundStyle(.white)
                    .position(x: 2*UIScreen.screenWidth/4, y: 19.25*UIScreen.screenHeight/30)
                    .font(.system(size: UIScreen.screenWidth/19.65, weight: .heavy, design: .rounded))
                
                /**************************************************************************************************************************************************************************************************************************************************************/
            
                
                /**************************************************************************************************************************************************************************************************************************************************************/
                
                
                
                /**************************************************************************************************************************************************************************************************************************************************************/
                
                //PM 2.5 Animation Button
                Button(action: {
                    self.First_isClicked = false
                    self.Second_isClicked = false
                    self.Third_isClicked = false
                    self.Fourth_isClicked = false
                    self.Fifth_isClicked = false
                    Number = bleManager.pm25Env
                    Unit = "PM 2.5"
                }){
                    //PM 2.5 Active Use check
                    if self.Sixth_isClicked {
                        Text(bleManager.pm25Env.formatted(.number.precision(.fractionLength(2))))
                            .foregroundStyle(.white)
                            .position(x: 2*UIScreen.screenWidth/4, y: 17*UIScreen.screenHeight/22)
                            .font(.system(size: 2*UIScreen.screenWidth/19.65, weight: .light, design: .serif))
                        //PM 2.5 Display Button
                            .background(RoundedRectangle(cornerRadius: UIScreen.screenWidth/15.72)
                                .fill(colorScheme == .light ? LinearGradient(stops: [
                                    Gradient.Stop(color: Color(red : 0.99, green: 0.56, blue: 0.11), location: 0.1),
                                    Gradient.Stop(color: .white, location: 0.9)
                                ], startPoint: .bottom, endPoint: .top):
                                        LinearGradient(stops: [
                                            Gradient.Stop(color: Color(red : 0.2, green: 0.11, blue: 0.7),
                                                          location: 0.1),
                                            Gradient.Stop(color: .black, location: 0.9)
                                        ], startPoint: .bottom, endPoint: .top))
                                    .frame(width: 2.75*UIScreen.screenWidth/3.144, height: UIScreen.screenWidth/3.144)
                                    .position(x: 2*UIScreen.screenWidth/4, y: 17*UIScreen.screenHeight/22)
                                    .shadow(color: colorScheme == .light ? .black : .white, radius: UIScreen.screenWidth/15.72))}
                    else {
                        Text(bleManager.pm25Env.formatted(.number.precision(.fractionLength(2))))
                            .foregroundStyle(.white)
                            .position(x: 2*UIScreen.screenWidth/4, y: 17*UIScreen.screenHeight/22)
                            .font(.system(size: 2*UIScreen.screenWidth/19.65, weight: .light, design: .serif))
                        //PM 2.5 Display Button
                            .background(RoundedRectangle(cornerRadius: UIScreen.screenWidth/15.72)
                                .fill(colorScheme == .light ? LinearGradient(stops: [
                                    Gradient.Stop(color: Color(red : 0.99, green: 0.56, blue: 0.11), location: 0.1),
                                    Gradient.Stop(color: .white, location: 0.9)
                                ], startPoint: .bottom, endPoint: .top):
                                        LinearGradient(stops: [
                                            Gradient.Stop(color: Color(red : 0.2, green: 0.11, blue: 0.7),
                                                          location: 0.1),
                                            Gradient.Stop(color: .black, location: 0.9)
                                        ], startPoint: .bottom, endPoint: .top))
                                    .frame(width: 2.75*UIScreen.screenWidth/3.144, height: UIScreen.screenWidth/3.144)
                                    .position(x: 2*UIScreen.screenWidth/4, y: 17*UIScreen.screenHeight/22))}
                    
                }
                //Gesture detection for PM 2.5 pressing or holding
                .simultaneousGesture(
                    LongPressGesture(minimumDuration: 0.01).onEnded({ _ in
                        self.Sixth_isClicked = true
                    })
                )
                //Style of PM 2.5 button
                .buttonStyle(PlainButtonStyle())
                //PM 2.5 Button Displayed Text
                Text("PM 2.5")
                    .foregroundStyle(.white)
                    .position(x: 2*UIScreen.screenWidth/4, y: 71.95*UIScreen.screenHeight/88)
                    .font(.system(size: UIScreen.screenWidth/19.65, weight: .heavy, design: .rounded))
                
                /**************************************************************************************************************************************************************************************************************************************************************/
                
                //Naviagtion Bar Display
                Rectangle()
                    .fill(colorScheme == .light ? Color(red : 0.8, green: 0.8, blue: 0.8) : Color(red : 0.1, green: 0.1, blue: 0.1))
                    .frame(width: UIScreen.screenWidth, height: 3*UIScreen.screenHeight/11)
                    .position(x: UIScreen.screenWidth/2, y: 99.5*UIScreen.screenHeight/100)
                
                /**************************************************************************************************************************************************************************************************************************************************************/
                
                
                //Naviagtion Bar Graph Display
                Button(action: {
                    //Take to the chart page
                    self.Values_isClicked = false
                    self.Export_isClicked = false
                    self.Export_Page = false
                    self.Value_Page = false
                }){
                    //Naviagtion Bar Graph Active Use check
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
                //Gesture detection for Naviagtion Bar Graph pressing or holding
                .simultaneousGesture(
                    LongPressGesture(minimumDuration: 0.01).onEnded({ _ in
                        self.Chart_isClicked = true
                        self.Graph_Page = true
                    })
                )
                //Style of Naviagtion Bar Graph button
                .buttonStyle(PlainButtonStyle())
                
                /**************************************************************************************************************************************************************************************************************************************************************/
                
                //Naviagtion Bar Value Display
                Button(action: {
                    //Take to the Value page
                    self.Chart_isClicked = false
                    self.Export_isClicked = false
                    self.Graph_Page = false
                    self.Export_Page = false
                }){
                    //Naviagtion Bar Value Active Use check
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
                //Gesture detection for Naviagtion Bar Value pressing or holding
                .simultaneousGesture(
                    LongPressGesture(minimumDuration: 0.01).onEnded({ _ in
                        self.Values_isClicked = true
                        self.Value_Page = true
                    })
                )
                //Style of Naviagtion Bar Value button
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
        else if Export_Page{
            Export()
        }
    }//End of View
}


#Preview {
    ContentView()
}

