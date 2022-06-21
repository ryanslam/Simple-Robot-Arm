function set_cartesian_position() {
    console.log("Client sending command MOVE_CARTESIAN_ABSOLUTE")
    x = document.getElementById("x_mm").value
    y = document.getElementById("y_mm").value
    use_traj = document.getElementById("use_cart_traj").checked
    traj_time = document.getElementById("cart_traj_time").value
    guessing_solution = document.getElementById("check_inverse_kinematic_solution").checked
    if (guessing_solution){
        t1_guess = document.getElementById("t1_inverse_kinematic_guess").value
        t2_guess = document.getElementById("t2_inverse_kinematic_guess").value
        socket.emit("set_cartesian_position", use_traj, traj_time, x, y, t1_guess, t2_guess);
    }else{
        socket.emit("set_cartesian_position", use_traj, traj_time, x, y);
    }
}