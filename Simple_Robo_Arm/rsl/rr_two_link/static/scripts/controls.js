function set_active_controller() {
    console.log("Client sending command SET_ACTIVE_CONTROLLER")
    controller_to_set = document.getElementById("controller_select").value
    console.log(controller_to_set);
    socket.emit("set_active_controller", controller_to_set)
}

function set_controller_gains() {
    console.log("Client sending command SET_CONTROLLER_GAINS")
    motor_id = document.getElementById("motor_id").value
    kp = document.getElementById("kp").value
    kd = document.getElementById("kd").value
    if(kp > 80 || kd > 80 || kp < 0 || kd < 0){
        alert("Inputted kp and kd values are out of range.\nViable Range: (0,80)")
    }
    else{
        socket.emit("set_controller_gains", motor_id, kp, 0, kd)
    }
}

function set_joint_position() {
    console.log("Client sending command MOVE_JOINTS_ABSOLUTE")
    t1 = document.getElementById("theta1").value
    t2 = document.getElementById("theta2").value
    use_traj = ""
    traj_time = "0"
    socket.emit("set_joint_position", use_traj, traj_time, t1, t2);

}