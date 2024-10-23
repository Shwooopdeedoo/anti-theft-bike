"use client"; // Make sure the component is treated as a client component

import { useState, useEffect } from 'react';
import Notifications from './notifications';
import MyMap from './map';

export default function Dashboard() {
    const [notifications, setNotifications] = useState([]);
    const [positions, setPositions] = useState([{ position: [30.6212, -96.3404], timestamp: new Date().toLocaleString() }]);
    const [selectedPosition, setSelectedPosition] = useState(null);

    // Function to parse the notification string and extract latitude and longitude
    const parseLocationMessage = (message) => {
        // Assuming the message format is: "Location: latitude, longitude"
        const parts = message.split(":")[1].split(",");
        const lat = parseFloat(parts[0].trim());
        const lng = parseFloat(parts[1].trim());
        return { lat, lng };
    };

    useEffect(() => {
        if (typeof window !== 'undefined') {
            const ws = new WebSocket('wss://ntfy.sh/JM4j2e0yT6akacaQ/ws');
            ws.onmessage = (event) => {
                const notification = JSON.parse(event.data);
                console.log('Received notification:', notification);
                setNotifications((prevNotifications) => [...prevNotifications, notification]);

                if (notification.message) {
                    // Parse the location from the message
                    const { lat, lng } = parseLocationMessage(notification.message);

                    // Ensure lat/lng are valid numbers before updating the state
                    if (!isNaN(lat) && !isNaN(lng)) {
                        const newPosition = {
                            position: [lat, lng],
                            timestamp: new Date().toLocaleString(),
                        };
                        // Add the new position to the state
                        setPositions((prevPositions) => [...prevPositions, newPosition]);
                        setSelectedPosition([lat, lng]); // Set the new position as selected
                    }
                }
            };

            ws.onerror = (error) => {
                console.error('WebSocket error:', error);
            };

            return () => {
                ws.close();
            };
        }
    }, []);

    const handleNotificationClick = (notification) => {
        if (notification.message) {
            const { lat, lng } = parseLocationMessage(notification.message);
            if (!isNaN(lat) && !isNaN(lng)) {
                const newPosition = [lat, lng];
                // When a notification is clicked, set that position as selected
                setSelectedPosition(newPosition);
            }
        }
    };

    return (
        <div className="relative w-full h-screen">
            <div className="absolute top-0 right-0 w-[20%] h-[45%] bg-yellow-100 border border-black z-10 overflow-y-auto">
                <Notifications notifications={notifications} onNotificationClick={handleNotificationClick} />
            </div>

            <div className="absolute inset-0 z-0">
                <MyMap positions={positions} zoom={18} selectedPosition={selectedPosition} />
            </div>
            <div className="z-20 absolute p-2 text-2xl rounded-tr-lg left-0 bottom-0 bg-black text-white px-4">
              CycleGuard
            </div>
        </div>
    );
}
