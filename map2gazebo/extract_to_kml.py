import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import NavSatFix
from pykml.factory import KML_ElementMaker as KML
from lxml import etree

# Function to read GPS data from ROS2 bag
def read_gps_data_from_bag(bag_path, time_interval_ns=5000000000):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    gps_data = []
    
    # Specify the type of message for deserialization
    navsatfix_msg_type = NavSatFix()

    last_timestamp = None

    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        if topic == '/summit_xl/ad/gnss/nav_sat_fix':
             # Only process data at intervals of `time_interval_ns` (5 seconds in nanoseconds)
            if last_timestamp is None or (timestamp - last_timestamp) >= time_interval_ns:
                # Deserialize the data into a NavSatFix message
                message = deserialize_message(data, NavSatFix)
                latitude = message.latitude
                longitude = message.longitude
                altitude = message.altitude
                gps_data.append((longitude, latitude, altitude))
                last_timestamp = timestamp
    
    return gps_data

# Function to create KML polygon from GPS data
def create_kml_polygon(gps_data, output_kml_path):
    # Define the KML structure with the GPS coordinates
    kml_doc = KML.kml(
        KML.Placemark(
            KML.name("GPS Path Polygon"),
            KML.Polygon(
                KML.outerBoundaryIs(
                    KML.LinearRing(
                        KML.coordinates(
                            ' '.join([f"{lon},{lat},{alt}" for lon, lat, alt in gps_data])
                        )
                    )
                )
            )
        )
    )

    # Write the KML data to a file
    with open(output_kml_path, 'w') as f:
        f.write(etree.tostring(kml_doc, pretty_print=True).decode())


# Path to your ROS2 bag
bag_path = "/scenario-simulation/rosbag2_2024_09_05-15_15_57/rosbag2_2024_09_05-15_15_57_0.db3"
output_kml_path = "./square_near_unisee.kml"

# Extract GPS data and create KML
gps_data = read_gps_data_from_bag(bag_path)
create_kml_polygon(gps_data, output_kml_path)

print(f"KML file saved to: {output_kml_path}")
