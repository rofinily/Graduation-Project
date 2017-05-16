function createWebSocket(host, port) {
    let url = 'ws://' + host + ':' + port;
    let socket = new WebSocket(url);

    socket.url = url;

    $('#connState').text('正在连接 ' + url);

    return socket;
}

function decorateSocket(socket) {
    socket.onopen = function (e) {
        console.log(e);

        $('#connState').text('成功连接至 ' + this.url);

        $('#trainFlag').prop('disabled', false)
            .prop('checked', false);
        $('button').each(function () {
            if (this.id === 'reconnect') {
                this.disabled = true;
            } else {
                this.disabled = false;
            }
        });

        requestLoop(this);
        requestOnce(this, MSG.GET_PERSONS);

        showCurFPS();
    }
    socket.onmessage = function (e) {
        //        console.log(e);
        handleMsg(e);
    }
    socket.onerror = function (e) {
        console.log(e);
        $('#connState').text('连接至 ' + this.url + ' 出错');
    }
    socket.onclose = function (e) {
        console.log(e);
        $('#connState').text('连接 ' + this.url + ' 已断开');

        $('#trainFlag').prop('disabled', true)
            .prop('checked', false);
        $('button').each(function () {
            if (this.id === 'reconnect') {
                this.disabled = false;
            } else {
                this.disabled = true;
            }
        });
    }

    return socket;
}

function decorateImageSocket(socket) {
    socket.onopen = function (e) {
        console.log(e);

        requestFixedLoop(this, MSG.RAW);
    }
    socket.onmessage = function (e) {
        //        console.log(e);
        handleMsg(e);
    }
    socket.onerror = function (e) {
        console.log(e);
    }
    socket.onclose = function (e) {
        console.log(e);
    }

    return socket;
}

function clearCanvas() {
    canvasCtx.clearRect(0, 0, imgWidth, imgHeight);
}

function handleMsg(e) {
    let j = JSON.parse(e.data);
    if (j.type === 'proceeded') {
        $('#faceImg').attr('src', j.content);
    } else if (j.type === 'persons') {
        let persons = j.content;
        // console.log(persons);
        // fitOptions(persons);
        fitPersonsTable(persons);
    } else if (j.type === 'detectedFaces') {
        clearCanvas();
        let boxes = j.boxes;
        for (let idx in boxes) {
            let box = boxes[idx];
            canvasCtx.strokeRect(box.left * imgScale,
                box.top * imgScale,
                (box.right - box.left) * imgScale,
                (box.bottom - box.top) * imgScale
            );
        }
    } else if (j.type === 'recognizedFace') {
        clearCanvas();
        let box = j.box;
        let label = j.label;
        canvasCtx.strokeRect(box.left * imgScale,
            box.top * imgScale,
            (box.right - box.left) * imgScale,
            (box.bottom - box.top) * imgScale
        );
        canvasCtx.fillText(label.name,
            box.left * imgScale,
            box.top * imgScale - 5
        );
    }
}

// function fitOptions(persons) {
//     $('#persons').empty();
//     for(let idx in persons) {
//         opt = document.createElement('option');
//         opt.value = idx;
//         opt.text = persons[idx];
//         opt.title = idx;
//         $('#persons').append(opt);
//     }
// }

// function onPersonClick(idx) {
//     $('#persons > option').each(function () {
//         if(!$('#trainFlag').prop('checked') && this.value === idx) {
//             $('#personName').val(this.text);
//             $('#personID').val(idx);
//         }
//     });
// }

function delPerson(thiz, identity) {
    if (confirm('确定删除吗？')) {
        let td = thiz.parentNode;
        let tr = td.parentNode;
        let tbody = tr.parentNode;
        tbody.removeChild(tr);

        let msg = MSG.DEL_PERSON;
        msg.identity = identity;
        requestOnce(socket, msg);
    }
}

function fitPersonsTable(persons) {
    let tbody = $('#personsTable > tbody');
    $(tbody).empty();
    for (let idx in persons) {
        let template =
            '<tr>\
                <td>{0}</td>\
                <td>{1}</td>\
                <td>\
                    <div class="btn-group" style="margin-right: 0.5ex;">\
                        <button name="retrain" class="btn btn-sm btn-primary" onclick="train({0}, \'{1}\')">继续学习</button>\
                        <button name="stopRetrain" class="btn btn-sm btn-danger" onclick="stopTrain()">停止</button>\
                    </div>\
                    <button class="btn btn-sm btn-danger" onclick="delPerson(this, {0})">删除</button>\
                </td>\
            </tr>';
        // console.log(format(template, idx, persons[idx], idx, persons[idx], idx));
        let tr = template.format(idx, persons[idx]);
        $(tbody).append(tr);
    }

    let nextId = parseInt(idx) + 1
    $('#personID').val(nextId);
    // let uploadImgHref = $('a#uploadImg').attr('href');
    // let tmp = uploadImgHref.substring(0, uploadImgHref.indexOf('?identity=') + '?identity='.length);
    // $('a#uploadImg').attr('href', tmp + nextId);
}

function requestOnce(socket, msg) {
    if (socket.readyState === socket.CLOSING || socket.readyState === socket.CLOSED) {
        return;
    }
    if (socket.readyState === socket.OPEN) {
        socket.send(JSON.stringify(msg));
        return;
    }
    setTimeout(requestOnce, msg.interval, socket, msg);
}

function requestLoop(socket) {
    if (socket.readyState === socket.CLOSING || socket.readyState === socket.CLOSED) {
        return;
    }
    if (socket.readyState === socket.OPEN) {
        socket.send(JSON.stringify(msg));
    }
    setTimeout(requestLoop, msg.interval / (fpsScale / 100 + 0.5), socket);
}

function requestFixedLoop(socket, msg) {
    if (socket.readyState === socket.CLOSING || socket.readyState === socket.CLOSED) {
        return;
    }
    if (socket.readyState === socket.OPEN) {
        socket.send(JSON.stringify(msg));
    }
    setTimeout(arguments.callee, msg.interval / (fpsScale / 100 + 0.5), socket, msg);
}

// function nothing() {
//     // interval = 1000;
//     // let obj = {
//     //     'cmd': 'nothing'
//     // }
//     // msg = JSON.stringify(obj);
//     msg = MSG.NOTHING;
//     // showCurFPS();
// }

function detect() {
    // interval = 500;
    // let obj = {
    //     'cmd': 'detect'
    // }
    // msg = JSON.stringify(obj);
    msg = MSG.DETECT;
}

// function train1(trainFlag) {
//     if(trainFlag) {
//         let name = $('#personName').val();
//         let identity = $('#personID').val();
//         if(!validateNameId(name, identity)) {
//             $('#trainFlag').prop('checked', false);
//             alert('名字或ID不合法');
//             return;
//         }

//         // interval = 750;
//         // let obj = {
//         //     'cmd': 'train',
//         //     'name': name,
//         //     'identity': identity
//         // };

//         // msg = JSON.stringify(obj);
//         msg = MSG.TRAIN;
//         msg.name = name;
//         msg.identity = identity;
//         showCurFPS();
//         return;
//     }

//     requestOnce(socket, MSG.GET_PERSONS);

//     // interval = 250;
//     // let obj = {
//     //     'cmd': 'stopTrain'
//     // }

//     requestOnce(socket, MSG.STOP_TRAIN);

//     // obj.cmd = 'raw';

//     // msg = JSON.stringify(obj);
//     msg = MSG.DETECT;
//     showCurFPS();
// }

function train(identity, name) {
    if (!validateNameId(name, identity)) {
        alert('名字或ID不合法');
        return;
    }

    msg = MSG.TRAIN;
    msg.name = name;
    msg.identity = identity;
}

function stopTrain() {
    requestOnce(socket, MSG.GET_PERSONS);

    requestOnce(socket, MSG.STOP_TRAIN);

    msg = MSG.DETECT;
}

function recognize() {
    // interval = 1000;
    // let obj = {
    //     'cmd': 'recognize'
    // }
    // msg = JSON.stringify(obj);
    msg = MSG.RECOGNIZE;
}

function validateNameId(name, identity) {
    if (name === '' || identity === '') {
        return false;
    }
    if (!/^\d+$/.test(identity)) {
        return false;
    }
    return true;
}

function doNothing() {
    setTimeout(clearCanvas, 1000);
    msg = MSG.NOTHING;
}

function showCurFPS() {
    let curFPS = 1 / (msg.interval / 1000);
    $('#curFPS').text(curFPS.toFixed(0));
}

function createCanvasCtx() {
    let canvasCtx = document.getElementById('faceCanvas').getContext('2d');
    canvasCtx.strokeStyle = 'white';
    canvasCtx.font = '2em serif';
    canvasCtx.fillStyle = 'white';
    return canvasCtx;
}

function getDataURLFromRGB(rgb) {
    let rgbLen = rgb.length;

    let canvas = $('<canvas/>').width(96).height(96)[0];
    let ctx = canvas.getContext("2d");
    let imageData = ctx.createImageData(96, 96);
    let data = imageData.data;
    let dLen = data.length;
    let i = 0, t = 0;

    for (; i < dLen; i += 4) {
        data[i] = rgb[t + 2];
        data[i + 1] = rgb[t + 1];
        data[i + 2] = rgb[t];
        data[i + 3] = 255;
        t += 3;
    }
    ctx.putImageData(imageData, 0, 0);

    return canvas.toDataURL("image/png");
}