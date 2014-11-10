#ifndef PThreadLocker_H
#define PThreadLocker_H

#include <pthread.h>

class PThreadLocker {
	public:
		PThreadLocker(pthread_mutex_t *_m) { m = _m; pthread_mutex_lock(m); }
		~PThreadLocker() { pthread_mutex_unlock(m); }

	private:
		pthread_mutex_t *m;
};

#endif
