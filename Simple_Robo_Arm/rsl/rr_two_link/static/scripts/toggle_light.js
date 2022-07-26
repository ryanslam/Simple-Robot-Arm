function toggle_light(){
    socket.emit("light_toggle", "Toggled");
}