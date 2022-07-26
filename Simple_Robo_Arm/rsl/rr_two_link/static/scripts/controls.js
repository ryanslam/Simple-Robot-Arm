function set_active_controller() {
    console.log("Client sending command SET_ACTIVE_CONTROLLER")
    controller_to_set = document.getElementById("controller_select").value
    socket.emit("set_active_controller", controller_to_set)
}

function set_controller_gains() {
    console.log("Client sending command SET_CONTROLLER_GAINS")
    motor_id = document.getElementById("motor_id").value
    kp = document.getElementById("kp").value
    // ki = document.getElementById("ki").value
    kd = document.getElementById("kd").value
    socket.emit("set_controller_gains", motor_id, kp, 0, kd)
}