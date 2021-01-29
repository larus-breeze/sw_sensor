/**
 * @file    FreeRTOS_wrapper.h
 * @brief   Comfortable versions of FreeRTOS's functions
 * @author  Dr. Klaus Schaefer schaefer@eit.h-da.de
 */
// doxygen mainpage
/**
 * @file    FreeRTOS_wrapper.h
 * @brief   Comfortable versions of FreeRTOS's functions
 * @author  Dr. Klaus Schaefer schaefer@eit.h-da.de
 * @mainpage
 *
 * Basic mechanisms (APIs)
 *
 * @see Class Task
 * @see suspend()
 * @see resume()
 * @see delay()
 * @see vTaskDelayUntil()
 * @see Class Semaphore
 * @see Class Mutex
 * @see Class Queue
 * @see Class timer
 *
 * Advanced mechanisms (APIs)
 *
 * @see Class RestrictedTask
 * @see Class active_object
 * @see Class MessageBuffer
 * @see Class StreamBuffer
 * @see Class event_group
 * @see Class timer
 * @see Class synchronous_timer
 *
 * Original FreeRTOS APIs
 * <a href="modules.html">Modules</a>
 */

#ifndef FREERTOSWRAPPER_H_
#define FREERTOSWRAPPER_H_

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "timers.h"
#include "event_groups.h"
#include "stream_buffer.h"
#include "message_buffer.h"
#include "my_assert.h"
#include "embedded_memory.h"
#include "string.h"

#define STANDARD_TASK_PRIORITY (3 + tskIDLE_PRIORITY)
#define INFINITE_WAIT portMAX_DELAY
#define NO_WAIT  (( TickType_t )0)

#define portSWITCH_TO_USER_MODE() __asm volatile ( " mrs r0, control \n orr r0, #1 \n msr control, r0 " ::: "r0", "memory" )

//! Suspend execution of this task
inline void suspend(void)
{
	vTaskSuspend(0);
}

//! Resume task execution
//! \param task_handle handle of FreeRTOS's task to be resumed
inline void resume(const TaskHandle_t task_handle)
{
	vTaskResume(task_handle);
}

//! Simple name for FreeRTOS's start function
inline void Start_FreeRTOS()
{
	vTaskStartScheduler();
}

// *******************************************************************************
#ifdef __cplusplus

#include "common.h"

//! Fast and easy initialization of auto variables
template<typename x> void wipe(x &that)
{
	for (uint8_t *ptr = (uint8_t *) &that; ptr < (uint8_t *) &that + sizeof(x);
			++ptr)
		*ptr = 0;
}

extern "C" BaseType_t xPortRaisePrivilege(void);
#define drop_privileges() portSWITCH_TO_USER_MODE()
#define acquire_privileges() xPortRaisePrivilege()

//! Template for a queue for arbitrary data
template<typename items>
class Queue
{
public:
//!  Queue constructor
//! \param  length Number of items that can be stored
	Queue(unsigned length)
	: the_queue( xQueueCreate(length, sizeof(items)))
	{
		ASSERT(the_queue != 0);
	}
protected:
	// support for Semaphore wanting item size = 0
	//!  protected alternate Queue constructor, only for use by semaphores
	Queue(unsigned length, unsigned size)
	: the_queue( xQueueCreate(length, size))
	{
		ASSERT(the_queue != 0);
	}

public:
	//!  Queue send method
	//! \param  item object to be sent
	//! \param TicksToWait maximum time to wait (optional)
	inline bool send(const items &item,
			unsigned TicksToWait = INFINITE_WAIT) const
	{
		return xQueueSend(the_queue, &item, TicksToWait) != pdFALSE;
	}

	//!  Queue receive method for use within ISR's
	//! \param  item object to be received
	inline bool receive_from_ISR(items &item) const
	{
		bool success;
		BaseType_t task_woken=false;
		success = xQueueReceiveFromISR(the_queue, &item, &task_woken);
		portEND_SWITCHING_ISR(task_woken);
		return success;
	}

	//!  Queue send method for use within ISR's
	//! \param  item object to be sent
	inline bool send_from_ISR(const items &item) const
	{
		bool success;
		BaseType_t task_woken=false;
		success = xQueueSendFromISR(the_queue, &item, &task_woken);
		portEND_SWITCHING_ISR(task_woken);
		return success;
	}

	//!  Queue receive method
	//! \param  item reference to an object to be received, will be overwritten
	//! \param TicksToWait maximum time to wait (optional ,default infinite wait)
	inline bool receive(items &item, unsigned TicksToWait = INFINITE_WAIT)
	{
		return xQueueReceive(the_queue, &item, TicksToWait) != pdFALSE;
	}
	//!  Queue flush method
	inline bool reset( void)
	{
		return xQueueReset( the_queue);
	}
	//!  Queue messages waiting method
	inline uint32_t messages_waiting(void)
	{
		return uxQueueMessagesWaiting(the_queue);
	}
	inline QueueHandle_t get_queue( void)
	{
		return the_queue;
	}
protected:
	QueueHandle_t the_queue; //!< freeRTOS's Queue handle
};

//! Template for a MessageBuffer for arbitrary objects
template<typename items>
class MessageBuffer
{
public:
//!  MessageBuffer constructor
//! \param  length Number of items that can be stored
	MessageBuffer(unsigned length)
	: xMessageBuffer( xMessageBufferCreate( sizeof(items) * length + 4 * length ))
	{
		ASSERT( xMessageBuffer != 0);
	}
	size_t send( const items & item, TickType_t xTicksToWait  = INFINITE_WAIT )
	{
	return xMessageBufferSend( xMessageBuffer, &item, sizeof(items), xTicksToWait );
	}
	size_t receive( items & item, TickType_t xTicksToWait = INFINITE_WAIT)
	{
	return xMessageBufferReceive( xMessageBuffer, &item, sizeof(items), xTicksToWait );
	}
	bool is_full( void)
	{
		return xMessageBufferIsFull(xMessageBuffer);
	}
	bool is_empty( void)
	{
		return xMessageBufferIsEmpty(xMessageBuffer);
	}
private:
	MessageBufferHandle_t xMessageBuffer;
};

//! class StreamBuffer
class StreamBuffer
{
public:
	//!  StreamBuffer constructor
	//! \param  length Number of items that can be stored
	//! \param  trigger_level Number of items that must be present to trigger the receiver
	StreamBuffer( size_t  length, size_t trigger_level=0)
	: buffer( xStreamBufferCreate( length, trigger_level ? trigger_level : length/2 ))
	{
		ASSERT( buffer != 0);
	}
	template < typename type> size_t send( const type * item, unsigned numb=1, TickType_t xTicksToWait  = INFINITE_WAIT )
	{
		return xStreamBufferSend( buffer, item, numb * sizeof(type), xTicksToWait );
	}
	template < typename type> size_t receive(  type * items, unsigned numb=1, TickType_t xTicksToWait  = INFINITE_WAIT )
	{
		return xStreamBufferReceive( buffer, items, sizeof(type)*numb, xTicksToWait);
	}
	template < typename type> size_t receive_bytes(  type * items, unsigned numb=1, TickType_t xTicksToWait  = INFINITE_WAIT )
	{
		return xStreamBufferReceive( buffer, items, sizeof(type)*numb, xTicksToWait);
	}
	template < typename type> unsigned receive_from_ISR( type * item, unsigned max_items=1)
	{
	  BaseType_t task_woken=0;
	  size_t bytes = xStreamBufferReceiveFromISR( buffer, item, max_items*sizeof(type), &task_woken );
	  portEND_SWITCHING_ISR(task_woken);
	  return bytes / sizeof(type);
	}
	size_t  bytes_full( void)
	{
		return xStreamBufferBytesAvailable( buffer);
	}
	size_t  bytes_empty( void)
	{
		return xStreamBufferSpacesAvailable( buffer);
	}
	bool is_full( void)
	{
		return xStreamBufferIsFull( buffer);
	}
	bool is_empty( void)
	{
		return xStreamBufferIsEmpty( buffer);
	}
private:
	StreamBufferHandle_t buffer;
};

//!  Counting and binary Semaphores
class Semaphore
{
public:
//!  Semaphore constructor
//! \param  length counting semaphore size (optional), length=1 or missing: create binary semaphore
	Semaphore(unsigned max_count=1, unsigned init_count=0)
	: sema( xSemaphoreCreateCounting( max_count, init_count))
	{
	}

	//!  signal method for use within tasks
	inline bool signal( void)
	{
		return xSemaphoreGive( sema);
	}

	//!  signal method for use within ISR's
	inline void signal_from_ISR(void) const
	{
		BaseType_t task_woken=false;
		xSemaphoreGiveFromISR( sema, &task_woken);
		portEND_SWITCHING_ISR(task_woken);
	}

	//!  wait method
	//! \param  TicksToWait maximum wait time (ticks)
	inline bool wait(unsigned TicksToWait = INFINITE_WAIT)
	{
		return xSemaphoreTake( sema, TicksToWait) != pdFALSE;
	}
private:
	SemaphoreHandle_t sema;
};

//! Mutex class
class Mutex
{
public:
	//!  Mutex constructor
	Mutex(void) :
			the_mutex(0)
	{
		the_mutex = xSemaphoreCreateMutex();
		ASSERT(the_mutex != 0);
	}
	//!  Lock method for Mutex
//! \param TicksToWait maximum time to wait to gain access (optional)
	inline bool lock(unsigned TicksToWait = INFINITE_WAIT)
	{
		return xSemaphoreTake(the_mutex, TicksToWait) != pdFALSE;
	}
//!  Release/Unlock method for Mutex
	inline void release(void)
	{
		xSemaphoreGive(the_mutex);
	}
//!  Release/Unlock method for Mutex
	inline void unlock(void)
	{
		xSemaphoreGive(the_mutex);
	}
private:
	SemaphoreHandle_t the_mutex; //!< FreeRTOS's SemaphoreHandle_t for the Mutex
};

//! Task class
class Task
{
public:
	//! Task constructor
	//! \param test_task code to be executed (TaskFunction_t)
	//! \param name tasks name (for debugging)
	//! \param stacksize size of stack in 32bit units
	//! \param parameters generic pointer to data
	//! \param priority tasks priority, task will always created privileged
	Task(TaskFunction_t code, char const * name = "TSK", uint16_t stack_size =
			configMINIMAL_STACK_SIZE, void * parameters = 0,
			unsigned priority = STANDARD_TASK_PRIORITY)
	: task_handle(0)
	{
		xTaskCreate(code, name, stack_size, parameters,
				priority | portPRIVILEGE_BIT, &task_handle);
		ASSERT(task_handle != 0);
	}

	//! Task handle getter function
	inline TaskHandle_t get_handle(void) const
	{
		return task_handle;
	}
//! Task handle getter function
	operator TaskHandle_t(void) const
	{
		return task_handle;
	}

	inline void suspend(void) const
	{
		vTaskSuspend(task_handle);
	}
	//! Resume task execution, callable ONLY from a task !
	inline void resume(void) const
	{
		vTaskResume(task_handle);
	}
	//! Resume task execution, callable ONLY from ISR !
	inline void resume_from_ISR(void) const
	{
		xTaskResumeFromISR(task_handle);
	}
	//! Wait for task notification
	//!
	//! \see xTaskNotifyWait
	static inline bool notify_wait( uint32_t BitsToClearOnEntry, uint32_t BitsToClearOnExit,
			                             uint32_t &NotificationValue,
			                             TickType_t TicksToWait=INFINITE_WAIT)
	{
		return xTaskNotifyWait( BitsToClearOnEntry, BitsToClearOnExit,
								&NotificationValue,
		                        TicksToWait ) != 0;
	}
	//! Notify task: Increment task notification value
	//!
	//! \see xTaskNotifyGive
	inline void notify_give( void)
	{
	 	 (void)xTaskNotifyGive( task_handle);
	}
	//! Notify task: Increment task notification value from ISR
	//!
	//! \see vTaskNotifyGiveFromISR
	void notify_give_from_ISR( void)
	{
		BaseType_t HigherPriorityTaskWoken=0;
		vTaskNotifyGiveFromISR( task_handle, &HigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(HigherPriorityTaskWoken);
	}
	//! Notify task: set event bits
	//!
	//! \see xTaskNotify
	inline bool notify( uint32_t value, eNotifyAction eAction )
	{
	 return xTaskNotify( task_handle, value, eAction);
	}
	//! Notify task: Set event bits from ISR
	//!
	//! \see xTaskNotifyFromISR
	inline bool notify_from_ISR( uint32_t value, eNotifyAction eAction) const
	{
		BaseType_t HigherPriorityTaskWoken=0;
		BaseType_t ret = xTaskNotifyFromISR( task_handle, value, eAction, &HigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(HigherPriorityTaskWoken); // sets pend_SV flag
		return ret != 0;
	}
	//! Clear task notification bits
	//!
	//! \see xTaskNotifySateClear
	inline void notify_clear( void)
	{
		xTaskNotifyStateClear( task_handle);
	}
protected:
	Task(void) :
			task_handle(0)
	{
	}
	TaskHandle_t task_handle; //!< FreeRTOS's internal task handle
};

static inline uint32_t notify_take( bool ClearCountOnExit=false, TickType_t TicksToWait=INFINITE_WAIT)
{
	return ulTaskNotifyTake( ClearCountOnExit, TicksToWait );
}

//! Resume task execution
//! \param thatone Task to be resumed
inline void resume(const Task &thatone)
{
	vTaskResume(thatone.get_handle());
}

/*! \brief RestrictedTask class
 *
 *  Create an unprivileged task with limited access rights.
 *
 *  A restricted task can only access it's automatic data.
 *
 *  Accessing global and static data is prohibited by the MPU.
 *
 *  Accessing COMMON data is allowed by default.
 */
class RestrictedTask: public Task
{
private:
	//! task constructor helper function
	void RestrictedTaskFromParameter(TaskParameters_t p, bool isSuspended)
	{
		if (p.puxStackBuffer == 0) // dynamically allocate stack memory if none given
		{
			p.puxStackBuffer = (StackType_t*) pvPortMallocAlignedMemory(
					p.usStackDepth * sizeof( portSTACK_TYPE),
					p.usStackDepth * sizeof( portSTACK_TYPE));
			ASSERT(p.puxStackBuffer != 0);
		}

		xTaskCreateRestricted(&p, &task_handle);
		if(isSuspended) {
			vTaskSuspend(task_handle);
		}
		ASSERT(task_handle != 0);
	}
public:
	//! RestrictedTask constructor
	//! \param p TaskParameters_t task parameters block
	//!
	//! If pointer to stack=0 new stack space will be allocated from FreeRTOs system memory pool
	//!
	//! Attention: Unprivileged tasks need aligned stack buffers !
	RestrictedTask(TaskParameters_t p, bool isSuspended = false)
	{
		RestrictedTaskFromParameter(p, isSuspended);
	}
	//! RestrictedTask constructor (simple)
	//! \param test_task code to be executed (TaskFunction_t)
	//! \param name tasks name (for debugging)
	//! \param stacksize size of stack in 32bit units
	//! \param parameters generic pointer to data
	//! \param priority tasks priority
	inline RestrictedTask(TaskFunction_t test_task, char const * name = "RES",
			uint16_t stack_size = configMINIMAL_STACK_SIZE, void * parameters =
					0, unsigned priority =
			STANDARD_TASK_PRIORITY, MemoryRegion_t *xRegions = 0, bool isSuspended = false)
	{
		TaskParameters_t p =
		{ test_task, name, stack_size, parameters, priority, 0,
			{
				{ COMMON_BLOCK, COMMON_SIZE, portMPU_REGION_READ_WRITE },
				{ 0, 0, 0 },
				{ 0, 0, 0 }
			}
		};
		if (xRegions != 0)
			memcpy(p.xRegions, xRegions,
					sizeof(MemoryRegion_t) * portNUM_CONFIGURABLE_REGIONS);
		RestrictedTaskFromParameter(p, isSuspended);
	}
};

//! helper function to compute a size that expressible as 2^n
inline unsigned fit_size( unsigned wanted_size)
{
	unsigned tile_size=32;
	while( tile_size < wanted_size) // compute smallest MPU section size fitting demand
		tile_size <<=1;
	return tile_size;
}

//! Object factory for active objects
//! \param runner class implementing a run() method
//! \param info argument type for the constructor of runner
template <class runner, typename info> class active_object
{
	typedef struct
	{
		info		task_info;		//!< arbitrary pointer for data for the runnable task
		runner ** 	pp_instance;	//!< address of pointer to the active object
	}task_info_block; 			//!< helper structure to provide task start information
public:
	//! constructor for the active object
	//! \param data Pointer to arbitrary data to pass to the runnable task

	active_object( info create_info, unsigned stacksize=configMINIMAL_STACK_SIZE,
				UBaseType_t priority=STANDARD_TASK_PRIORITY,
				char const * task_name="A_O")
		: init_data( { create_info, &the_instance}),
		the_instance(0),
		task(
				{
					(TaskFunction_t) start,
					task_name,
					// max. possible stack fitting into the MPU page in 32bit word units
					(uint16_t)fit_size( stacksize*sizeof( portSTACK_TYPE) + sizeof(runner)),
					(void *)&init_data,
					priority | portPRIVILEGE_BIT,
					// align the stack to the bottom of our MPU page
					// the stack top is the bottom of our (*this) object
					0,
					{
							{COMMON_BLOCK,COMMON_SIZE,portMPU_REGION_READ_WRITE},
							{0,0,0},
							{0,0,0}
					}
				}
			)
	{};
	//! Getter function for the runnable object
	inline runner & instance( void) const
	{
		return *the_instance;
	}
	//! Getter function for the executing task
	inline const Task & get_task( void) const
	{
		return task;
	}
private:
	//! this method is run within the created task
	//!
	//! it creates the active object and execute it's run() method
	static void start( task_info_block *data)
	{
		runner the_object( data->task_info); // create object
		*(data->pp_instance) = &the_object;  // remember instance within active_object
		drop_privileges();					 // go unprivileged
		the_object.run(); 	 				 // go running
		ASSERT( 0); 						 // run() should not return !!
		suspend();  						 // just in case it does anyway ...
	}
	task_info_block init_data;	//!< all information for task start
	runner *the_instance;		//!< pointing to the instance of the active object
	RestrictedTask task;		//!< the task belonging to the active object
};

//! Delay task execution
//! \param time delay time in ticks
inline void delay(TickType_t time)
{
	vTaskDelay(time);
}

//! \brief wrapper around FreeRTOS's timers
//!
//! Objects of this class allow spawning timed callback functions
class timer
{
public:
	timer(TickType_t period, TimerCallbackFunction_t callback) :
			timer_ID(0)
	{
		timer_ID = xTimerCreate(0, period, 0, 0, callback);
		ASSERT(timer_ID != 0);
	}
	~timer( void)
	{
		BaseType_t result = xTimerDelete( timer_ID, INFINITE_WAIT );
		ASSERT( result != 0);
	}
	void start(void)
	{
		(void) xTimerStart(timer_ID, INFINITE_WAIT);
	}
	void stop(void)
	{
		(void) xTimerStop(timer_ID, INFINITE_WAIT);
	}
	//! \brief start a timer from ISR context
	void start_from_ISR(void)
	{
		BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
		(void) xTimerStartFromISR(timer_ID, &pxHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(pxHigherPriorityTaskWoken);
	}
	void reset(void)
	{
		(void) xTimerReset(timer_ID, INFINITE_WAIT);
	}
private:
	TimerHandle_t timer_ID;
};

//! \brief wrapper around vTaskDelayUntil(...)
//!
//! Objects of this class allow precise periodic and long-term clock-synchronous activities
class synchronous_timer
{
public:
	synchronous_timer(TickType_t period = 0) :
			TimeIncrement(period), PreviousWakeTime( xTaskGetTickCount())
	{
	}
	//! synchronize to periodic timer
	inline bool sync( void)
	{
		ASSERT(TimeIncrement != 0);
		// make sure there is a time greater than zero to wait
		bool ok = xTaskGetTickCount() < (PreviousWakeTime + TimeIncrement);
		vTaskDelayUntil( &PreviousWakeTime, TimeIncrement);
		return ok;
	}
	//! re-synchronize to present RTOS system time
	inline void reset(void)
	{
		PreviousWakeTime = xTaskGetTickCount();
	}
	inline void re_synchronize(TickType_t time)
	{
		PreviousWakeTime = time;
	}
	//! synchronous delay using DelayUntil(...)
	inline void delay(TickType_t duration)
	{
		vTaskDelayUntil(&PreviousWakeTime, duration);
	}
private:
	TickType_t TimeIncrement;
	TickType_t PreviousWakeTime;
};

//! \brief Class event_group
//!
//! \see EventGroup
class event_group
{
public:
	//! \brief Default constructor for an empty event_group object
	event_group( void)
	: EventGroup( xEventGroupCreate())
	{};
	//! \brief wait for a bit or group of bits to become set
	//! \param BitsToWaitFor bitmask for bits to wait for
	//! \param ClearOnExit if true: clear all bits from BitsToWaitFor on exit, otherwise don't clear any bits
	//! \param WaitForAllBits if true: wait for all bits, otherwise wait for a single bit
	//! \return the bits that are presently set on function return
	inline EventBits_t wait_bits(
			const EventBits_t BitsToWaitFor,
            bool ClearOnExit,
            bool WaitForAllBits,
            TickType_t TicksToWait = INFINITE_WAIT)
		{
			return xEventGroupWaitBits(
				EventGroup,
				BitsToWaitFor,
				ClearOnExit,
				WaitForAllBits,
				TicksToWait );
		}
	//! \brief set bits, not to be called from ISR's
	inline EventBits_t set_bits( const EventBits_t BitsToSet)
		{
			return xEventGroupSetBits( EventGroup, BitsToSet);
		}
	//! \brief set bits, only to be called from ISR's
	void set_bits_from_ISR( const EventBits_t BitsToSet)
	{
		BaseType_t HigherPriorityTaskWoken=0;
		bool success = xEventGroupSetBitsFromISR(
		           EventGroup,
				   BitsToSet,
		           &HigherPriorityTaskWoken );
		ASSERT( success);
		portEND_SWITCHING_ISR( HigherPriorityTaskWoken);
	}
	//! \brief return current value
	inline EventBits_t get_bits( void)
	{
		return xEventGroupGetBits( EventGroup);
	}
	//! \brief clear a bit or a set of bits
	inline EventBits_t clear_bits( const EventBits_t BitsToclear)
	{
		return xEventGroupClearBits( EventGroup, BitsToclear);
	}
	//! \brief Report events and synchronize tasks (also called "rendezvous")
	//!
	//! Atomically set bits (flags) within an RTOS event group,
	//! then wait for a combination of bits to be set
	//! \param uxBitsToSet bits to set when function is called
	//! \param BitsToWaitFor bits to wait for before return
	//! \return the set of bits that are set the moment the function returns
	inline EventBits_t syncronize(
            const EventBits_t BitsToSet,
            const EventBits_t BitsToWaitFor,
            TickType_t xTicksToWait = INFINITE_WAIT)
	{
		return xEventGroupSync( EventGroup,
				BitsToSet,
				BitsToWaitFor,
				xTicksToWait );
	}
private:
	EventGroupHandle_t EventGroup; //!< FreeRTOS's internal EventGroup handle

};

#endif // cplusplus
#endif /* FREERTOSWRAPPER_H_ */
