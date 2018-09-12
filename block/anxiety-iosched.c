/*
 * Anxiety I/O scheduler
 * Copywrite (C) 2018 Draco (Tyler Nijmeh) <tylernij@gmail.com>
 */
#include <linux/blkdev.h>
#include <linux/elevator.h>
#include <linux/bio.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>

struct anxiety_data {
	struct list_head queue;
};

static inline void anxiety_merged_requests(struct request_queue *q, struct request *rq, struct request *next) {
	list_del_init(&next->queuelist);
}

static struct request *anxiety_choose_request(struct anxiety_data *mdata, int data_dir) {
	struct list_head *head;
	list_for_each_prev(head, &mdata->queue) {
		struct request *rq = (struct request *) head;
		const int sync = rq_is_sync(rq);
		const int read = rq_data_dir(rq);

		//return rq_entry_fifo(async[data_dir].next);
	}

	return NULL;
}

static int anxiety_dispatch(struct request_queue *q, int force) {
	struct anxiety_data *nd = q->elevator->elevator_data;
	struct request *rq;

	rq = list_first_entry_or_null(&nd->queue, struct request, queuelist);
	if (!rq)
		return 0;

	list_del_init(&rq->queuelist);
	elv_dispatch_sort(q, rq);
	return 1;
}

static inline void anxiety_add_request(struct request_queue *q, struct request *rq) {
	list_add_tail(&rq->queuelist, &((struct anxiety_data *) q->elevator->elevator_data)->queue);
}

static inline struct request *anxiety_former_request(struct request_queue *q, struct request *rq) {
	if (rq->queuelist.prev == &((struct anxiety_data *) q->elevator->elevator_data)->queue)
		return NULL;
	return list_prev_entry(rq, queuelist);
}

static inline struct request *anxiety_latter_request(struct request_queue *q, struct request *rq) {
	if (rq->queuelist.next == &((struct anxiety_data *) q->elevator->elevator_data)->queue)
		return NULL;
	return list_next_entry(rq, queuelist);
}

static int anxiety_init_queue(struct request_queue *q, struct elevator_type *e) {
	struct anxiety_data *nd;
	struct elevator_queue *eq;

	eq = elevator_alloc(q, e);
	if (!eq)
		return -ENOMEM;

	nd = kmalloc_node(sizeof(*nd), GFP_KERNEL, q->node);
	if (!nd) {
		kobject_put(&eq->kobj);
		return -ENOMEM;
	}
	eq->elevator_data = nd;

	INIT_LIST_HEAD(&nd->queue);
	spin_lock_irq(q->queue_lock);
	q->elevator = eq;
	spin_unlock_irq(q->queue_lock);
	return 0;
}

static void anxiety_exit_queue(struct elevator_queue *e) {
	BUG_ON(!list_empty(&((struct anxiety_data *) e->elevator_data)->queue));
}

static struct elevator_type elevator_anxiety = {
	.ops = {
		.elevator_merge_req_fn		= anxiety_merged_requests,
		.elevator_dispatch_fn		= anxiety_dispatch,
		.elevator_add_req_fn		= anxiety_add_request,
		.elevator_former_req_fn		= anxiety_former_request,
		.elevator_latter_req_fn		= anxiety_latter_request,
		.elevator_init_fn		= anxiety_init_queue,
		.elevator_exit_fn		= anxiety_exit_queue,
	},
	.elevator_name = "anxiety",
	.elevator_owner = THIS_MODULE,
};

static int __init anxiety_init(void) {
	return elv_register(&elevator_anxiety);
}

static void __exit anxiety_exit(void) {
	elv_unregister(&elevator_anxiety);
}

module_init(anxiety_init);
module_exit(anxiety_exit);

MODULE_AUTHOR("Draco (Tyler Nijmeh)");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Anxiety IO scheduler");
