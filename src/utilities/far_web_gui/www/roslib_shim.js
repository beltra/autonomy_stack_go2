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
      this._cb = null;       // wrapped callback registered with ros
      this._rawCb = null;    // original user callback
      this._throttleTimer = null;
    }

    subscribe(cb) {
      // Client-side throttle + queue fallback.
      //
      // rosbridge *should* honour throttle_rate and queue_length, but some
      // versions ignore them.  The full roslibjs library has its own
      // client-side throttle; we replicate the same "throttle with latest
      // delivery" pattern here:
      //
      //  - While inside a throttle window, incoming messages are queued
      //    (up to queue_length, default 1 = keep only the newest).
      //  - When the window expires, the most recent queued message is
      //    delivered to the user callback.
      //
      // This guarantees:
      //   1. The callback never fires more often than throttle_rate ms.
      //   2. The callback always receives the *freshest* available data.
      //   3. No message is silently dropped without a newer replacement.

      let wrappedCb = cb;

      if (typeof this.throttle_rate === 'number' && this.throttle_rate > 0) {
        const minInterval = this.throttle_rate;
        const maxQueue = (typeof this.queue_length === 'number' && this.queue_length > 0)
          ? this.queue_length : 1;
        let pending = [];       // queued messages waiting for delivery
        let lastFired = 0;      // timestamp of last callback invocation
        let timer = null;       // scheduled delivery timer
        const self = this;

        wrappedCb = function throttledWithQueue(msg) {
          const now = Date.now();

          // If we're outside the throttle window, deliver immediately.
          if (now - lastFired >= minInterval) {
            lastFired = now;
            pending.length = 0;
            if (timer) { clearTimeout(timer); timer = null; }
            cb(msg);
            return;
          }

          // Inside the throttle window — queue the message.
          pending.push(msg);
          // Enforce queue_length: keep only the newest N messages.
          while (pending.length > maxQueue) pending.shift();

          // Schedule delivery of the newest queued message at the end of
          // the current throttle window (if not already scheduled).
          if (!timer) {
            const remaining = minInterval - (now - lastFired);
            timer = setTimeout(function deliverQueued() {
              timer = null;
              if (pending.length > 0) {
                const latest = pending[pending.length - 1];
                pending.length = 0;
                lastFired = Date.now();
                cb(latest);
              }
            }, remaining);
            self._throttleTimer = timer;
          }
        };
      }

      this._rawCb = cb;
      this._cb = wrappedCb;
      const topic = this.name;
      const ros = this.ros;

      if (!ros._topicSubs.has(topic)) ros._topicSubs.set(topic, []);
      ros._topicSubs.get(topic).push(wrappedCb);

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
      // Clear any pending throttle timer.
      if (this._throttleTimer) {
        clearTimeout(this._throttleTimer);
        this._throttleTimer = null;
      }
      this._cb = null;
      this._rawCb = null;
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
