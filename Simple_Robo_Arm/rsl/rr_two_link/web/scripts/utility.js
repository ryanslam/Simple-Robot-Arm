function reconnect_motors() {
    console.log("Client sending command RECONNECT_MOTORS")
    socket.emit("reconnect_motors")
}

function go_home() {
    console.log("Client sending command GO_HOME")
    socket.emit("home");
}

function disable_torque() {
    console.log("Client sending command DISABLE_TORQUE")
    socket.emit("torque_disable");
}

function enable_torque() {
    console.log("Client sending command ENABLE_TORQUE")
    socket.emit("torque_enable")
}