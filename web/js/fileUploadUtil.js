function createFileReader() {
    let fr = new FileReader();

    fr.onloadstart = function (e) {
        // console.log(e);
    }
    fr.onload = function (e) {
        // console.log(e);
        loadedSize += e.loaded;

        let content = e.currentTarget.result;
        let msg = {
            'cmd': 'uploadPic',
            'name': $('#personName').val(),
            'identity': $('#personID').val(),
            'content': content,
            'uploadedSize': loadedSize
        }
        requestOnce(JSON.stringify(msg));

        if (loadedSize == totalSize) {
            $('#uploadState').text('全部读取完毕，正在上传...');
            $('button').prop('disabled', true);
            // $('#submit').prop('disabled', false);
        }
    }
    fr.onprogress = function (e) {
        $('#progress').attr('value', loadedSize + e.loaded);
    }
    fr.onabort = function (e) {
        console.log(e)
        $('#uploadState').text('上传已中断')
    }
    fr.onerror = function (e) {
        console.log(e)
    }

    return fr;
}

function createWebSocket(host, port) {
    let url = 'ws://' + host + ':' + port;
    let socket = new WebSocket(url);

    $('#connState').text('正在连接 ' + url);

    socket.onopen = function (e) {
        console.log(e);
        $('#connState').text('成功连接至 ' + url);
        // $('button').prop('disabled', false);
        $('button').each(function () {
            if (this.id == 'reconnect') {
                this.disabled = true;
            } else {
                this.disabled = false;
            }
        });
    }
    socket.onmessage = function (e) {
        //        console.log(e);
        handleMsg(e);
    }
    socket.onerror = function (e) {
        console.log(e);
        $('#connState').text('连接至 ' + url + ' 出错');
    }
    socket.onclose = function (e) {
        console.log(e);
        $('#connState').text('连接 ' + url + ' 已断开');
        // $('button').prop('disabled', true);
        // $('button#reconnect').prop('disabled', false);
        $('button').each(function () {
            if (this.id == 'reconnect') {
                this.disabled = false;
            } else {
                this.disabled = true;
            }
        });
    }

    return socket;
}

function handleMsg(e) {
    let j = JSON.parse(e.data);
    if (j.type == 'uploadedSize') {
        $('#progress').attr('value', j.uploadedSize);

        if (j.uploadedSize == totalSize) {
            msg = {
                'cmd': 'allFileUploaded'
            }
            requestOnce(JSON.stringify(msg));
            $('#uploadState').text('全部上传完毕, 正在学习...');
        }
    } else if (j.type == 'allPicTrained') {
        $('#uploadState').text('学习完毕');
        $('button').each(function () {
            if (this.id == 'reconnect') {
                this.disabled = true;
            } else {
                this.disabled = false;
            }
        });
    }

}

function requestOnce(msg) {
    if (socket.readyState == socket.CLOSING || socket.readyState == socket.CLOSED) {
        return;
    }
    if (socket.readyState == socket.OPEN) {
        socket.send(msg);
        return;
    }
    setTimeout(requestOnce, 100, msg);
}

function read(fr, f) {
    if (fr.readyState == fr.LOADING) {
        setTimeout(read, 100, fr, f);
        return;
    }
    fr.readAsDataURL(f);
}

function validateNameId(name, identity) {
    if (name == '' || identity == '') {
        return false;
    }
    if (!/^\d+$/.test(identity)) {
        return false;
    }
    return true;
}