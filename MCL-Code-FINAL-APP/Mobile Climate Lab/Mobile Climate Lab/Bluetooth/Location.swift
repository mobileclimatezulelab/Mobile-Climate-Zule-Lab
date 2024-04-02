//
//  Location.swift
//  Mobile Climate Lab
//
//  Created by Eslam Othman on 2024-03-12.
//
import Foundation
import CoreLocation
import MapKit

class DiscoverViewController : UIViewController, ObservableObject,  CLLocationManagerDelegate {
    
    @IBOutlet weak var map: MKMapView!
    var locationManager: CLLocationManager!
    
    override func viewDidLoad()
    {
        super.viewDidLoad()
        
        if (CLLocationManager.locationServicesEnabled())
        {
            locationManager = CLLocationManager()
            locationManager.delegate = self
            locationManager.desiredAccuracy = kCLLocationAccuracyBest
            locationManager.requestWhenInUseAuthorization()
            locationManager.requestAlwaysAuthorization()
            locationManager.startUpdatingLocation()
        }
    }
    
    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        if let location = locations.last {
            let coordinate = CLLocationCoordinate2D(latitude: location.coordinate.latitude, longitude: location.coordinate.longitude)
            
            SensorDataManager.shared.currentLocation = location

            // Update the map region to center on the new location
            let region = MKCoordinateRegion(center: coordinate, span: MKCoordinateSpan(latitudeDelta: 0.01, longitudeDelta: 0.01))
            self.map.setRegion(region, animated: true)
            
            // Create a minimalistic MKPointAnnotation at the location
            let dotAnnotation = MKPointAnnotation()
            dotAnnotation.coordinate = coordinate
            
            self.map.addAnnotation(dotAnnotation)
        }
    }
}
