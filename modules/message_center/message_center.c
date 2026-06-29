#include "message_center.h"
#include "string.h"

/* message_center是fake head node,是方便链表编写的技巧,这样就不需要处理链表头的特殊情况 */
static Publisher_t message_center = {
    .topic_name = "Message_Manager",
    .first_subs = NULL,
    .next_topic_node = NULL};

/* 全局互斥锁，保护所有 pub/sub 操作 */
static SemaphoreHandle_t msg_center_mutex = NULL;

void MessageCenterInit(void)
{
    /* 在调度器启动前创建互斥锁 */
    msg_center_mutex = xSemaphoreCreateMutex();
}

/* ---------- 内部辅助函数（调用前已加锁） ---------- */

static void CheckName(char *name)
{
    if (strnlen(name, MAX_TOPIC_NAME_LEN + 1) >= MAX_TOPIC_NAME_LEN)
    {
        /* 话题名超出长度限制 —— 死循环便于调试定位 */
        while (1)
            ;
    }
}

static void CheckLen(uint8_t len1, uint8_t len2)
{
    if (len1 != len2)
    {
        /* 相同话题的消息长度不同 —— 死循环便于调试定位 */
        while (1)
            ;
    }
}

/* ---------- 公开 API（内部加锁） ---------- */

Publisher_t *PubRegister(char *name, uint8_t data_len)
{
    CheckName(name);

    xSemaphoreTake(msg_center_mutex, portMAX_DELAY);

    Publisher_t *node = &message_center;
    while (node->next_topic_node)
    {
        node = node->next_topic_node;
        if (strcmp(node->topic_name, name) == 0)
        {
            CheckLen(data_len, node->data_len);
            node->pub_registered_flag = 1;
            xSemaphoreGive(msg_center_mutex);
            return node;
        }
    }

    /* 在链表尾部创建新的话题 */
    node->next_topic_node = (Publisher_t *)pvPortMalloc(sizeof(Publisher_t));
    memset(node->next_topic_node, 0, sizeof(Publisher_t));
    node->next_topic_node->data_len = data_len;
    strcpy(node->next_topic_node->topic_name, name);

    xSemaphoreGive(msg_center_mutex);
    return node->next_topic_node;
}

Subscriber_t *SubRegister(char *name, uint8_t data_len)
{
    CheckName(name);

    xSemaphoreTake(msg_center_mutex, portMAX_DELAY);

    Publisher_t *pub = &message_center;
    while (pub->next_topic_node)
    {
        pub = pub->next_topic_node;
        if (strcmp(pub->topic_name, name) == 0)
        {
            CheckLen(data_len, pub->data_len);
            goto create_sub;
        }
    }

    /* 话题不存在，先创建发布者节点（延迟创建） */
    pub->next_topic_node = (Publisher_t *)pvPortMalloc(sizeof(Publisher_t));
    memset(pub->next_topic_node, 0, sizeof(Publisher_t));
    pub->next_topic_node->data_len = data_len;
    strcpy(pub->next_topic_node->topic_name, name);
    pub = pub->next_topic_node;

create_sub:
    /* 创建新的订阅者结点 */
    Subscriber_t *ret = (Subscriber_t *)pvPortMalloc(sizeof(Subscriber_t));
    memset(ret, 0, sizeof(Subscriber_t));
    ret->data_len = data_len;
    for (size_t i = 0; i < QUEUE_SIZE; ++i)
    {
        ret->queue[i] = pvPortMalloc(data_len);
    }

    /* 如果是第一个订阅者 */
    if (pub->first_subs == NULL)
    {
        pub->first_subs = ret;
        xSemaphoreGive(msg_center_mutex);
        return ret;
    }

    /* 遍历到订阅者链表尾部 */
    Subscriber_t *sub = pub->first_subs;
    while (sub->next_subs_queue)
    {
        sub = sub->next_subs_queue;
    }
    sub->next_subs_queue = ret;

    xSemaphoreGive(msg_center_mutex);
    return ret;
}

uint8_t SubGetMessage(Subscriber_t *sub, void *data_ptr)
{
    xSemaphoreTake(msg_center_mutex, portMAX_DELAY);

    if (sub->temp_size == 0)
    {
        xSemaphoreGive(msg_center_mutex);
        return 0;
    }
    memcpy(data_ptr, sub->queue[sub->front_idx], sub->data_len);
    sub->front_idx = (sub->front_idx + 1) % QUEUE_SIZE;
    sub->temp_size--;

    xSemaphoreGive(msg_center_mutex);
    return 1;
}

uint8_t PubPushMessage(Publisher_t *pub, void *data_ptr)
{
    xSemaphoreTake(msg_center_mutex, portMAX_DELAY);

    Subscriber_t *iter = pub->first_subs;
    while (iter)
    {
        if (iter->temp_size == QUEUE_SIZE)
        {
            /* 队列已满，丢弃最老的数据 */
            iter->front_idx = (iter->front_idx + 1) % QUEUE_SIZE;
            iter->temp_size--;
        }
        /* 将数据复制到队列尾部 */
        memcpy(iter->queue[iter->back_idx], data_ptr, pub->data_len);
        iter->back_idx = (iter->back_idx + 1) % QUEUE_SIZE;
        iter->temp_size++;

        iter = iter->next_subs_queue;
    }

    xSemaphoreGive(msg_center_mutex);
    return 1;
}
