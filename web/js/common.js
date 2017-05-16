// function format() {
// 	let args = [];
// 	for (let attr in arguments) {
// 		if (attr === 'callee') {
// 			break;
// 		}
// 		args.push(arguments[attr]);
// 	}

// 	if (args.length < 1) {
// 		throw Error('no string to replace');
// 	}

// 	if (args.length < 2) {
// 		return args[0];
// 	}

// 	let str = args[0];
// 	let placeholders = args.slice(1);

// 	let result = str.replace(/{.+?}/g, function (match) {
// 		match = match.slice(1, match.length - 1);

// 		if (!/\d+/.test(match)) {
// 			throw Error('index can only be integer greater than 0, got ' + match);
// 		}

// 		let idx = parseInt(match);
// 		if (idx > placeholders.length) {
// 			throw Error('out of placeholders index, got ' + idx);
// 		}
// 		return placeholders[idx];
// 	});

// 	return result;
// }

String.prototype.format = function () {
	//无参数，返回源字符串
	if (arguments.length === 0) {
		return this;
	}

	let placeholders = [];
	for (let i = 0; i < arguments.length; i++) {
		placeholders.push(arguments[i]);
	}

	let result = null;

	//考虑`{}`的情况
	if (!/{[^{}]+?}/g.test(this)) {
		let matchIndex = 0;
		result = this.replace(/{}/g, function (match) {
			if (matchIndex > placeholders.length - 1) {
				throw Error('out of placesholders range, got {} out of {}'.format(matchIndex, placeholders.length - 1));
			}
			return placeholders[matchIndex++]
		})

		return result;
	}

	//考虑`{}`和`{string}`的情况
	result = this.replace(/{[^{}]*?}/g, function (match, position) {
		match = match.slice(1, match.length - 1);

		//`{}`的情况
		if (match === '') {
			throw Error('placeholder `{integer}` must have index, got none in position ' + position);
		}

		//`{非数字string}`的情况
		if (!/\d+/.test(match)) {
			throw Error('index can only be integer greater than 0, got {} in position {}'.format(match, position));
		}

		//`{integer}`的情况
		let idx = parseInt(match);
		if (idx > placeholders.length - 1) {
			throw Error('out of placeholders range, got {} out of {}'.format(idx, placeholders.length - 1));
		}
		return placeholders[idx];
	});

	return result;
};