window.addEventListener("keydown", keysdown, false);
window.addEventListener("keyup", keyuped, false);

var keys = [];
var start = false;
var cmd_x = 0;
var cmd_y = 0;
var cmd_z = 0;
var twist = new ROSLIB.Message({
    linear : {
        x : 0.0,
        y : 0.0,
        z : 0.0
    },
    angular : {
        x : 0.0,
        y : 0.0,
        z : 0.0
    }
});

function KeyboardState(state) {
    start = state;
}

function keysdown(e) {
    if (start == true) {
        keys[e.keyCode] = true;
        if (keys[87] && keys[68]) {
            //w+d
            cmd_x = 1;
            cmd_y = -1;
            cmd_z = 0;
        }else if (keys[87] && keys[65]){
            //w+a
            cmd_x = 1;
            cmd_y = 1;
            cmd_z = 0;
        }else if (keys[83] && keys[68]){
            //s+d
            cmd_x = -1;
            cmd_y = -1;
            cmd_z = 0;
        }else if (keys[83] && keys[65]) {
            //s+a
            cmd_x = -1;
            cmd_y = 1;
            cmd_z = 0;
        }else if (keys[87]) {
            //w
            cmd_x = 1;
            cmd_y = 0;
            cmd_z = 0;
        }else if (keys[68]) {
            //d
            cmd_x = 0;
            cmd_y = -1;
            cmd_z = 0;
        }else if (keys[83]) {
            //s
            cmd_x = -1;
            cmd_y = 0;
            cmd_z = 0;
        }else if (keys[65]) {
            //a
            cmd_x = 0;
            cmd_y = 1;
            cmd_z = 0;
        }else if (keys[81]) {
            //q
            cmd_x = 0;
            cmd_y = 0;
            cmd_z = 1;
        }else if (keys[69]) {
            //e
            cmd_x = 0;
            cmd_y = 0;
            cmd_z = -1;
        }
        twist = new ROSLIB.Message({
            linear : {
              x : 0.05*cmd_x*speed,
              y : 0.05*cmd_y*speed,
              z : 0.0
            },
            angular : {
              x : 0.0,
              y : 0.0,
              z : 0.05*cmd_z*speed
            }
        });
        pub_cmdVel.publish(twist)
    }
}

function releasebutton(state) {

    // switch(state){
    //     case 81:
    //         console.log(81);
    //         break;
    //     case 69:
    //         console.log(69);
    //         break;
    //     case 87:
    //         console.log(87);
    //         break;
    //     case 65:
    //         console.log(65);
    //         break;
    //     case 83:
    //         console.log(83);
    //         break;
    //     case 68:
    //         console.log(68);
    //         break;
    //     default:
    //         console.log(00)
    // }

    twist = new ROSLIB.Message({
      linear : {
        x : 0.0,
        y : 0.0,
        z : 0.0
      },
      angular : {
        x : 0.0,
        y : 0.0,
        z : 0.0
      }
    });
    pub_cmdVel.publish(twist)
}

function keyuped(e) {
    if (start) {
        if (keys[e.keyCode] == true) releasebutton(e.keyCode);
        keys[e.keyCode] = false;
    }
}
