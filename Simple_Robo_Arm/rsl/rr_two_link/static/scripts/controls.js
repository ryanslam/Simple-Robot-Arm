function set_active_controller() {
    console.log("Client sending command SET_ACTIVE_CONTROLLER")
    controller_to_set = document.getElementById("controller_select").value;
    kp_0 = document.getElementById("kp").value;
    kd_0 = document.getElementById("kd").value;
    kp_1 = document.getElementById("kp").value;
    kd_1 = document.getElementById("kd").value;
    kps = toString([kp_0, kp_1]);
    kds = toString([kd_0, kd_1]);
    if(controller_to_set === '0'){
        disbale_gains_section(false);
        socket.emit("set_active_controller", controller_to_set);
    }
    else if(controller_to_set === '4'){
        disbale_gains_section(true);
        console.log('sending kps:' + kps + ' and \n\tkds:' +kds)
        socket.emit("set_active_controller", controller_to_set, kps, kds);
    }
    else{
        disbale_gains_section(true);
        socket.emit("set_active_controller", controller_to_set)
    }
}

// Old active controller
// function set_active_controller() {
    //   console.log("Client sending command SET_ACTIVE_CONTROLLER")
    //   controller_to_set = document.getElementById("controller_select").value
    //   if (controller_to_set === '0') {
    //     disbale_gains_section(false);
    //     socket.emit("set_active_controller", controller_to_set)
    //   }
    //   else {
    //     disbale_gains_section(true);
    //     socket.emit("set_active_controller", controller_to_set)
    //   }

function disbale_gains_section(value){
    var childNodes = document.getElementById("actuator-position-controller-gain-config").getElementsByTagName("*");
    for (var node of childNodes){
        node.disabled = value;
        if (node.tagName == "BUTTON"){
          continue;
        }
        // Grey out the disabled elements.
        if (value==true){
            node.style.color = "#D3D3D3";
        }
        // Reset the original element color.
        else{
            node.style.color = "#000000";
        }
    }
}


function set_controller_gains() {
    console.log("Client sending command SET_CONTROLLER_GAINS")
    motor_id = document.getElementById("motor_id").value
    ki_check = document.getElementById("check_enable_kis")
    kp = document.getElementById("kp").value
    kd = document.getElementById("kd").value
    if(ki_check){
        socket.emit("set_controller_gains", motor_id, kp, ki, kd)
    }
    else{
        socket.emit("set_controller_gains", motor_id, kp, '0,0', kd)
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