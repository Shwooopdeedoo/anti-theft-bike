import React, { useRef, useEffect } from 'react';
import { ScrollArea, ScrollBar } from "./components/ui/scroll-area";

export default function Notifications({ notifications, onNotificationClick }) {
    const scrollRef = useRef(null);

    useEffect(() => {
        if (scrollRef.current) {
            scrollRef.current.scrollTop = scrollRef.current.scrollHeight;
        }
    }, [notifications]);

    return (
        <div className="items-center text-center font-mono">
            <h1 className="text-xl bg-red-900 py-4 text-white font-bold border-b-4 border-black">Location History</h1>
            <ScrollArea className="h-64 w-full overflow-auto relative">
                <div className="pr-4 pl-4" ref={scrollRef}>
                    {notifications.length > 0 ? (
                        <ul className="flex flex-col">
                            {notifications.map((notification, index) => (
                                <div 
                                    key={index}
                                    className="py-2 border-b-[1px] border-black cursor-pointer"
                                    onClick={() => onNotificationClick(notification)}
                                >
                                    <li>{notification.body || notification.message}</li>
                                </div>
                            ))}
                        </ul>
                    ) : (
                        <p className="p-2">No notifications yet</p>
                    )}
                </div>
                <ScrollBar orientation="vertical" />
            </ScrollArea>
        </div>
    );
}
