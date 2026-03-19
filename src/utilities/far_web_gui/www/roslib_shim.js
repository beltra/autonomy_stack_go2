/*
 * Lightweight ROSLIB shim for FAR Web GUI.
 * Implements the subset used by app.js over rosbridge websocket:
 *   ROSLIB.Ros, ROSLIB.Topic, ROSLIB.Service,
 *   ROSLIB.Message, ROSLIB.ServiceRequest
 */

(function initRoslibShim(global) {
  'use strict';

  if (global.ROSLIB) {
    // Full roslib already loaded (e.g. from CDN). Keep it.
    return;
  }

  function makeEventEmitter(target) {
    const listeners = new Map();
    target.on = function on(evt, cb) {
      if (!listeners.has(evt)) listeners.set(evt, []);
      listeners.get(evt).push(cb);
    };
    target.off = function off(evt, cb) {
      const arr = listeners.get(evt);
      if (!arr) return;
      const idx = arr.indexOf(cb);
      if (idx >= 0) arr.splice(idx, 1);
    };
    target._emit = function emit(evt, ...args) {
      const arr = listeners.get(evt);
      if (!arr) return;
      arr.slice().forEach((fn) => {
        try { fn(...args); } catch (e) { console.error(e); }
      });
    };
    return target;
  }

  class Ros {
    constructor(opts) {
      makeEventEmitter(this);
      this.url = opts && opts.url ? opts.url : '';
      this.socket = null;
      this._id = 1;
      this._topicSubs = new Map();   // topic => [callbacks]
      this._serviceCalls = new Map(); // id => { successCb, errorCb, timer }
      this._connect();
    }

    _failPendingServiceCalls(reason) {
      const pending = Array.from(this._serviceCalls.values());
      this._serviceCalls.clear();
      pending.forEach((entry) => {
        if (entry && entry.timer) clearTimeout(entry.timer);
        if (entry && typeof entry.errorCb === 'function') {
          try { entry.errorCb(reason); } catch (e) { console.error(e); }
        }
      });
    }

    _connect() {
      this.socket = new WebSocket(this.url);
      this.socket.onopen = () => this._emit('connection');
      this.socket.onerror = (err) => {
        this._emit('error', err);
        this._failPendingServiceCalls(new Error('ROS websocket error'));
      };
      this.socket.onclose = () => {
        this._emit('close');
        this._failPendingServiceCalls(new Error('ROS websocket closed'));
      };
      this.socket.onmessage = (evt) => {
        let msg;
        try {
          msg = JSON.parse(evt.data);
        } catch (e) {
          console.warn('Invalid rosbridge message', e);
          return;
        }

        if (msg.op === 'publish' && msg.topic) {
          const list = this._topicSubs.get(msg.topic);
          if (!list) return;
          list.slice().forEach((cb) => {
            try { cb(msg.msg); } catch (e) { console.error(e); }
          });
          return;
        }

        if (msg.op === 'service_response' && msg.id) {
          const entry = this._serviceCalls.get(msg.id);
          if (entry) {
            this._serviceCalls.delete(msg.id);
            if (entry.timer) clearTimeout(entry.timer);
            const ok = msg.result !== false;
            if (ok) {
              try { entry.successCb(msg.values || {}); } catch (e) { console.error(e); }
            } else if (typeof entry.errorCb === 'function') {
              try { entry.errorCb(msg.values || {}); } catch (e) { console.error(e); }
            }
          }
        }
      };
    }

    _nextId(prefix) {
      const id = `${prefix || 'id'}:${this._id++}`;
      return id;
    }

    _send(obj) {
      if (!this.socket || this.socket.readyState !== WebSocket.OPEN) {
        console.warn('ROS websocket is not open, dropping message:', obj && obj.op ? obj.op : obj);
        return false;
      }
      this.socket.send(JSON.stringify(obj));
      return true;
    }

    close() {
      if (this.socket) this.socket.close();
    }
  }

  class Topic {
    constructor(opts) {
      this.ros = opts.ros;
      this.name = opts.name;
      this.messageType = opts.messageType;
      this.throttle_rate = opts.throttle_rate;
      this.queue_length = opts.queue_length;
      this.compression = opts.compression;
      this._cb = null;
    }

    subscribe(cb) {
      this._cb = cb;
      const topic = this.name;
      const ros = this.ros;

      if (!ros._topicSubs.has(topic)) ros._topicSubs.set(topic, []);
      ros._topicSubs.get(topic).push(cb);

      const payload = {
        op: 'subscribe',
        topic,
        type: this.messageType,
      };
      if (typeof this.throttle_rate === 'number') payload.throttle_rate = this.throttle_rate;
      if (typeof this.queue_length === 'number') payload.queue_length = this.queue_length;
      if (typeof this.compression === 'string') payload.compression = this.compression;
      ros._send(payload);
    }

    unsubscribe() {
      const topic = this.name;
      const ros = this.ros;
      const list = ros._topicSubs.get(topic);
      if (list && this._cb) {
        const idx = list.indexOf(this._cb);
        if (idx >= 0) list.splice(idx, 1);
      }
      if (!list || list.length === 0) {
        ros._topicSubs.delete(topic);
        ros._send({ op: 'unsubscribe', topic });
      }
      this._cb = null;
    }

    publish(msg) {
      this.ros._send({ op: 'publish', topic: this.name, msg: msg || {} });
    }
  }

  class Service {
    constructor(opts) {
      this.ros = opts.ros;
      this.name = opts.name;
      this.serviceType = opts.serviceType;
    }

    callService(request, callback, errback, options) {
      const ros = this.ros;
      const id = ros._nextId('call_service');
      const timeoutMs = options && typeof options.timeout === 'number' ? options.timeout : 10000;
      const successCb = callback || function noop() {};
      const errorCb = errback || function noop() {};
      const timer = setTimeout(() => {
        const pending = ros._serviceCalls.get(id);
        if (!pending) return;
        ros._serviceCalls.delete(id);
        try { pending.errorCb(new Error('Service call timeout')); } catch (e) { console.error(e); }
      }, timeoutMs);

      ros._serviceCalls.set(id, { successCb, errorCb, timer });
      const sent = ros._send({
        op: 'call_service',
        id,
        service: this.name,
        type: this.serviceType,
        args: request || {},
      });

      if (!sent) {
        clearTimeout(timer);
        ros._serviceCalls.delete(id);
        try { errorCb(new Error('ROS websocket is not open')); } catch (e) { console.error(e); }
      }
    }
  }

  class Message {
    constructor(values) {
      Object.assign(this, values || {});
    }
  }

  class ServiceRequest {
    constructor(values) {
      Object.assign(this, values || {});
    }
  }

  global.ROSLIB = {
    Ros,
    Topic,
    Service,
    Message,
    ServiceRequest,
  };
})(window);
