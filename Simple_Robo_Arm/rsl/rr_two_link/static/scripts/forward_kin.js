function set_joint_position() {
    console.log("Client sending command MOVE_JOINTS_ABSOLUTE")
    t1 = document.getElementById("theta1").value
    t2 = document.getElementById("theta2").value
    guessing_solution = document.getElementById("check_forward_kinematic_solution").checked
    use_traj = document.getElementById("use_joint_traj").checked
    traj_time = document.getElementById("joint_traj_time").value
    if (guessing_solution){
        x_guess = document.getElementById("x_forward_kinematic_guess").value
        y_guess = document.getElementById("y_forward_kinematic_guess").value
        socket.emit("set_joint_position", use_traj, traj_time, t1, t2, x_guess, y_guess);
    }else{
        socket.emit("set_joint_position", use_traj, traj_time, t1, t2);
    }
}

function controls_refresher_demo() {
    console.log("Client sending command CONTROLS_REFRESHER_DEMO")
    socket.emit("controls_refresher")
}

function workspace_demo() {
    console.log("Client sending command WORKSPACE_DEMO")
    theta_1_min = document.getElementById("theta_1_min_workspace").value
    theta_1_max = document.getElementById("theta_1_max_workspace").value
    if(parseFloat(theta_1_max) <= 90 || parseFloat(theta_1_max) >= -90 || parseFloat(theta_1_min) >= -90 || parseFloat(theta_1_min) <= 90){
        document.getElementById("guessed_solution").innerHTML = "";
        socket.emit("workspace_demo", theta_1_min, theta_1_max);
    }
    else{
        document.getElementById("guessed_solution").innerHTML = "Error, the max and minimum values lie within the range of (-100,100)";
    }
}