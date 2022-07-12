function jacobian_demo() {
    console.log("Client sending command JACOBIAN_DEMO")
    theta2 = document.getElementById("theta_2_jacobian_demo").value
    //socket.emit("jacobian_demo", theta2)
    socket.emit("jacobian_demo", theta2)
}