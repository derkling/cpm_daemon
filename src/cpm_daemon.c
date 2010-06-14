/**     cpm_daemon.c
*
*		A task control daemon for the Constrained Power Management framework
*
*      Copyright 2009 Massari Giuseppe <joe.massanga@gmail.com>
*
*      This program is free software; you can redistribute it and/or modify
*      it under the terms of the GNU General Public License as published by
*      the Free Software Foundation; either version 2 of the License, or
*      (at your option) any later version.
*
*      This program is distributed in the hope that it will be useful,
*      but WITHOUT ANY WARRANTY; without even the implied warranty of
*      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*      GNU General Public License for more details.
*
*      You should have received a copy of the GNU General Public License
*      along with this program; if not, write to the Free Software
*      Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
*      MA 02110-1301, USA.
*/

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <assert.h>
#include <errno.h>
#include <string.h>
#include <ctype.h>
#include <libxml/parser.h>
#include <libxml/tree.h>
#include <sysfs/libsysfs.h>
#include <sysfs/dlist.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <setjmp.h>
#include <signal.h>
#include <syslog.h>
#include <dirent.h>
#include <unistd.h>
#include <linux/connector.h>
#include <linux/netlink.h>
#include <linux/cn_proc.h>


/**************************
 * Error return codes     *
 **************************/
#define E_NOROOT		1
#define E_CONFIG 		2
#define E_NO_CPM		3
#define E_NO_SWMS		4
#define E_LIST			5
#define E_SYSFS			6
#define E_NL_CONNECT		7


/************************
 * Utility macros 	*
 ************************/

/* Macros for netlink/proc connection purpose	*/
#define SEND_MESSAGE_LEN (NLMSG_LENGTH(sizeof(struct cn_msg) + sizeof(enum proc_cn_mcast_op)))
#define RECV_MESSAGE_LEN (NLMSG_LENGTH(sizeof(struct cn_msg) + sizeof(struct proc_event)))

#define SEND_MESSAGE_SIZE    (NLMSG_SPACE(SEND_MESSAGE_LEN))
#define RECV_MESSAGE_SIZE    (NLMSG_SPACE(RECV_MESSAGE_LEN))

#define max(x,y) ((y)<(x)?(x):(y))
#define min(x,y) ((y)>(x)?(x):(y))
#define BUFF_SIZE (max(max(SEND_MESSAGE_SIZE, RECV_MESSAGE_SIZE), 1024))

/************************
 * Constant data	*
 ************************/

/* Paths... */
#define CPM_SYSFS_PATH			"/kernel/cpm"
#define CPM_USER_IF			"/xps"
#define CPM_SWMS_PATH 	 		"/sys/kernel/cpm/swms/"
#define CPM_ASSERT_XP_PATH		"/sys/kernel/cpm/xps/assert"
#define CPM_NEW_XP_PATH 		"/sys/kernel/cpm/xps/control"
#define CPM_XPS_PATH			"/sys/kernel/cpm/xps/"

/* Info macros	*/
#define PROG_NAME		"CPM Execution profiles control daemon"
#define AUTHOR			"Giuseppe Massari <joe.massanga@gmail.com>"
#define LICENSE			"GPL"

/* String size constants	*/
#define CPM_NAME_LEN		32
#define BUF_MAX_DIM_ALLOW	256
#define CPM_COMM_LEN		100

/* Command macro indexes	*/
#define CMD_ASSERT	0
#define CMD_DEASSERT	1

#define CTRL_ADD	0
#define CTRL_REMOVE	1

/*****************************************
 * Data types				 *
 *****************************************/

/**
 * constraint_data - constraint on a System-wide Metric
 *
 * @param swm_id - the ID of the SWM (read from cpm-core)
 * @param name - the name of the SWM
 * @param value - the constraint value
 * @param min - the lower bound (read from cpm-core)
 * @param max - the upper bound (read from cpm-core)
 *
 * @note ...
 */
struct constraint_data
{
    unsigned int swm_id;
    char swm_name[CPM_NAME_LEN];
    /* Value specified in the configuration	*/
    unsigned int value;
    /* Bounds defined by the platform in CPM	*/
    unsigned int min;
    unsigned int max;

    char type;
    char composition;
    char permission;
    char constraint[20];
};

/**
 * xp_data - Execution Profile data
 *
 * @param name - the name of the execution profile
 * @param constraints - the list of SWMs upon which assert constraints
 */
struct xp_data
{
    char name[CPM_NAME_LEN];
    struct dlist *constraints;
};

/**
 * pid_data - struct for manage the PIDs
 */
struct pid_data
{
    pid_t pid;
    pid_t tgid;
};


/**
 * app_data - Application data
 *
 * @param name - the name of the application (exe file)
 * @param priority - a static priority for cpm-core scheduling purpose
 * @param xps - the list of the execution profiles specified by the application
 * @param pid_instances - the list of all pids (if there's more than one instance of the application running)
 */
struct app_data
{
    char name[CPM_NAME_LEN];
    int priority;
    struct dlist *xps;
    struct dlist *pid_instances;
};


/*******************************************
 *  		Global variables           *
 *******************************************/

/* Commands for execution profiles assertion	*/
const char *assert_comm[] =
{
    "assert",
    "deassert"
};

/* Commands execution profiles registration 	*/
const char *control_comm[] =
{
    "add",
    "remove"
};

struct dlist *xps_list;			/* The list of Application Running Profile */
struct dlist *applications_list;	/* The list of the Application specified in the config file */

char config_file_path[1024];		/* Path for the configuration file	*/
short int enable_debug = 0;		/* Debug mode or not ?	*/
char sysfs_path[SYSFS_PATH_MAX];	/* Mount path of SysFS 	*/

int default_prog_prio = 5;		/* Default priority for any applications	*/
int ret;				/* Return value	*/
sigjmp_buf g_jmp;
xmlDocPtr doc; 				/* The resulting document tree from config.xml	*/
DIR *dirp;				/* Descriptor for directory cheching purpose	*/


/* Macro for printing debug messages	*/
#define print_debug(message...) 	if (enable_debug) fprintf(stderr, message);


/****************************************
 * Function prototypes			*
 ****************************************/

void handle_proc_events (struct cn_msg *cn_hdr);
int load_configuration (const char *);
void register_xp (xmlNodePtr);
void register_application (xmlNodePtr);
struct xp_data *xp_registered (char *);
struct app_data *get_app_by_name (char *);
struct app_data *get_app_by_pid(struct pid_data *);
void assert_xp (struct app_data *, struct pid_data *, int);
void handle_intr(int);

int app_pid_compare(void *, void *);
int swm_name_compare(void *, void *);
int xp_name_compare(void *, void *);
int xp_name_match(void *, void *);
int app_name_compare(void *, void *);
int app_name_match(void *, void *);
void print_help();

/** main()
 * ****************************************/
int main(int argc, char** argv)
{
    int sk_nl;					/* Netlink Socket */
    struct sockaddr_nl my_nla, kern_nla, from_nla;
    socklen_t from_nla_len;
    char buff[BUFF_SIZE];
    struct nlmsghdr *nl_hdr;			/* Netlink Message Header	*/
    struct cn_msg *cn_hdr;			/* Connector Message Header	*/
    enum proc_cn_mcast_op *mcop_msg;
    size_t recv_len = 0;
    pid_t pid;
    int option;

    /* Is the root user ?	*/
    if (getuid() != 0)
    {
        fprintf(stderr, "[CPM] You must be root to start this daemon.\n");
        return E_NOROOT;
    }

    /* Manage command-line options	*/
    opterr = 0;

    if (argc < 2)
    {
        print_help();
        exit(EXIT_FAILURE);
    }

    while ((option = getopt(argc, argv, "c:p:dvh")) != -1)
    {
        switch (option)
        {
        case 'c':
            strncpy(config_file_path, optarg, sizeof(config_file_path));
            break;
        case 'p':
            default_prog_prio = atoi(optarg);
            break;
        case 'd':
            enable_debug = 1;
            break;
        case 'v':
            fprintf(stderr, "%s\nVersion: %s\nAuthor: %s\nLicense:%s\n", PROG_NAME, VERSION, AUTHOR, LICENSE);
            exit(EXIT_SUCCESS);
        case 'h':
            print_help();
            exit(EXIT_SUCCESS);
        case '?':
            if (optopt == 'c')
                fprintf (stderr, "Option -%c requires a file path as argument.\n", optopt);
            else if (optopt == 'p')
                fprintf (stderr, "Option -%c requires an integer value.\n", optopt);
            else if (isprint (optopt))
                fprintf (stderr, "Unknown option `-%c'.\n", optopt);
            exit(EXIT_FAILURE);
        }
    }

    if (strcmp(config_file_path, "") == 0)
    {
        print_help();
        exit(EXIT_FAILURE);
    }

    /* Has sysfs been mounted ?	*/
    if (sysfs_get_mnt_path(sysfs_path, SYSFS_PATH_MAX) != 0)
    {
        fprintf(stderr, "[CPM] SysFS not mounted.\n");
        return E_SYSFS;
    }
    /* Had CPM been load ?	*/
    strcat(sysfs_path, CPM_SYSFS_PATH);
    strcat(sysfs_path, CPM_USER_IF);

    if (!(dirp = opendir(sysfs_path)))
    {
        fprintf(stderr, "[CPM] CPM user interface not loaded on the system.\n");
        return E_NO_CPM;
    }
    closedir(dirp);

    /* Create the list of the XPs and of the Applications	*/
    xps_list = dlist_new(sizeof(struct xp_data));
    applications_list = dlist_new(sizeof(struct app_data));

    /* Load configuration file with XPs and Applications	*/
    if ((ret = load_configuration(config_file_path)) != 0)
        return ret;

    /* Open the system log	*/
    openlog("CPM daemon", LOG_PID, LOG_LPR | LOG_DAEMON);

    /* Daemonize...		*/
    if (!enable_debug)
    {
        /* Detach a new process to became a daemon */
        if ((pid = fork()) < 0)
        {
            perror("CPM - demonizing");
            exit(EXIT_FAILURE);
        }
        else if (pid > 0)
        {
            /* Waits for initialization to complete and parent exit */
            sleep(3);
            exit(EXIT_SUCCESS);
        }
        /* Change file mode */
        umask(0);

        /* Set a new session	*/
        if (setsid() < 0)
        {
            perror("CPM - session id");
            exit(EXIT_FAILURE);
        }

        /* Move to root directory */
        if ((chdir("/")) < 0)
        {
            perror("CPM - root directory");
            exit(EXIT_FAILURE);
        }

        /* Close any standard input/output */
        close(STDIN_FILENO);
        close(STDOUT_FILENO);
        close(STDERR_FILENO);
    }

    /* Start writing to syslog... */
    syslog(LOG_INFO, "started");

    /* Build up the connection for the netlink proc connector	*/
    sk_nl = socket(PF_NETLINK, SOCK_DGRAM, NETLINK_CONNECTOR);
    if (sk_nl == -1)
    {
        syslog(LOG_ERR, "Error in netlink proc connector socket opening.\n");
        closelog();
        close(sk_nl);
        return E_NL_CONNECT;
    }
    my_nla.nl_family = AF_NETLINK;
    my_nla.nl_groups = CN_IDX_PROC;
    my_nla.nl_pid = getpid();

    kern_nla.nl_family = AF_NETLINK;
    kern_nla.nl_groups = CN_IDX_PROC;
    kern_nla.nl_pid = 1;

    if (bind(sk_nl, (struct sockaddr *) &my_nla, sizeof(my_nla)) == -1)
    {
        syslog(LOG_ERR, "Error in netlink proc connector socket binding.\n");
        closelog();
        close(sk_nl);
        return E_NL_CONNECT;
    }

    nl_hdr = (struct nlmsghdr *) buff;
    cn_hdr = (struct cn_msg *) NLMSG_DATA(nl_hdr);
    mcop_msg = (enum proc_cn_mcast_op *) &cn_hdr->data[0];

    if (sigsetjmp(g_jmp, SIGINT) != 0)
    {
        print_debug("sending proc connector: PROC_CN_MCAST_IGNORE... ");
        memset(buff, 0, sizeof(buff));
        *mcop_msg = PROC_CN_MCAST_IGNORE;
    }
    else
    {
        print_debug("sending proc connector: PROC_CN_MCAST_LISTEN... ");
        memset(buff, 0, sizeof(buff));
        *mcop_msg = PROC_CN_MCAST_LISTEN;
        signal(SIGINT, handle_intr);
    }
    /* ...fill the netlink header */
    nl_hdr->nlmsg_len = SEND_MESSAGE_LEN;
    nl_hdr->nlmsg_type = NLMSG_DONE;
    nl_hdr->nlmsg_flags = 0;
    nl_hdr->nlmsg_seq = 0;
    nl_hdr->nlmsg_pid = getpid();
    /* ...fill the connector header */
    cn_hdr->id.idx = CN_IDX_PROC;
    cn_hdr->id.val = CN_VAL_PROC;
    cn_hdr->seq = 0;
    cn_hdr->ack = 0;
    cn_hdr->len = sizeof(enum proc_cn_mcast_op);

    if (send(sk_nl, nl_hdr, nl_hdr->nlmsg_len, 0) != nl_hdr->nlmsg_len)
    {
        syslog(LOG_ERR, "failed to send proc connector mcast ctl op!\n");
        closelog();
        close(sk_nl);
        return E_NL_CONNECT;
    }

    print_debug("sent\n");
    if (*mcop_msg == PROC_CN_MCAST_IGNORE)
    {
        closelog();
        close(sk_nl);
        exit(EXIT_SUCCESS);
    }
    print_debug("Reading process events from proc connector.\n"
                "Hit Ctrl-C to exit\n");

    memset(buff, 0, sizeof(buff));
    struct nlmsghdr *nlh = (struct nlmsghdr *) buff;

    /* Loop on netlink proc connector messages */
    for (memset(buff, 0, sizeof(buff)), from_nla_len = sizeof(from_nla);
            ; memset(buff, 0, sizeof(buff)), from_nla_len = sizeof(from_nla))
    {

        memset(nlh, 0, NLMSG_SPACE(BUFF_SIZE));
        memcpy(&from_nla, &kern_nla, sizeof(from_nla));
        recv_len = recvfrom(sk_nl, buff, BUFF_SIZE, 0,(struct sockaddr*) &from_nla, &from_nla_len);
        if (recv_len < 1)
            continue;

        while (NLMSG_OK(nlh, recv_len))
        {
            cn_hdr = NLMSG_DATA(nlh);
            if (nlh->nlmsg_type == NLMSG_NOOP)
                continue;

            if ((nlh->nlmsg_type == NLMSG_ERROR) || (nlh->nlmsg_type == NLMSG_OVERRUN))
                break;

            handle_proc_events(cn_hdr);

            if (nlh->nlmsg_type == NLMSG_DONE)
                break;

            nlh = NLMSG_NEXT(nlh, recv_len);
        }
    }

    return ret;
}

/**
 * handle_proc_events()
 * @brief Handle kernel connector event messages
 * @param cn_hdr The connector header
 */
void handle_proc_events(struct cn_msg *cn_hdr)
{
    int fd;
    char fname[255], log_msg[255], status[255];
    char task_name[100];
    struct app_data *ret_app;
    struct pid_data pid_target;
    struct pid_data *pid_inst;
    struct pid_data *curr_pids;

    strncpy(fname, " ", sizeof(fname));
    strncpy(status, " ", sizeof(status));
    strncpy(task_name, " ", sizeof(task_name));

    /* Get data from proc connector */
    struct proc_event *ev = (struct proc_event *) cn_hdr->data;

    /* Get the name of the process/application from procfs 	*/
    snprintf(fname, sizeof(fname), "/proc/%d/status", ev->event_data.exec.process_pid);

    if ((fd = open(fname, O_RDONLY)) > 0)
        ret = read(fd, status, sizeof(status));
    close(fd);

    sscanf(status, "Name: %s", task_name);

    /* Which event occurred ?	*/
    switch (ev->what)
    {
    case PROC_EVENT_EXEC:

        print_debug("EXEC:pid=%d,tgid=%d\t[%s]\n", ev->event_data.exec.process_pid, ev->event_data.exec.process_tgid, task_name);

        if ((ret_app = get_app_by_name(task_name)))
        {
            /* Once get the application struct...	*/
            sprintf(log_msg, "(%d) '%s' in execution\n", ev->event_data.exec.process_pid, task_name);
            syslog(LOG_INFO, "%s", log_msg);

            /* Set PIDs info */
            pid_inst = malloc(sizeof(struct pid_data));
            pid_inst->pid = ev->event_data.exec.process_pid;
            pid_inst->tgid = ev->event_data.exec.process_tgid;

            /* Append the PIDs instance	*/
            dlist_insert_sorted(ret_app->pid_instances, (struct pid_data *) pid_inst, app_pid_compare);

            /* ...and assert the XP	*/
            assert_xp(ret_app, pid_inst, CMD_ASSERT);
        }
        break;

    case PROC_EVENT_EXIT:

        pid_target.pid = ev->event_data.exec.process_pid;
        pid_target.tgid = ev->event_data.exec.process_tgid;

        print_debug("EXIT: pid=%d,%d\texit code=%d\n", pid_target.pid, pid_target.tgid, ev->event_data.exit.exit_code);

        if ((ret_app = get_app_by_pid(&pid_target)))
        {
            /* If the task is a known application...	*/
            sprintf(log_msg, "(%d) exits\n", pid_target.pid);
            syslog(LOG_INFO, "%s", log_msg);

            /* Retrieve the PID instance...	*/
            pid_target.pid = ev->event_data.exec.process_pid;
            pid_target.tgid = ev->event_data.exec.process_tgid;

            /* Release the assertion	*/
            assert_xp(ret_app, &pid_target, 1);

            /* And remove the PIDs instance	*/
            dlist_for_each_data(ret_app->pid_instances, curr_pids, struct pid_data)
            if (curr_pids->pid == pid_target.pid)
                dlist_delete(ret_app->pid_instances, CMD_DEASSERT);
        }
        break;

    default:
        break;
    }
}


/**
 * load_configuration()
 *
 * @brief Load the configuration file
 */
int load_configuration(const char *config_file_path)
{
    xmlNodePtr cur;		/* Current pointer for the XML tree exploration	*/
    xmlNodePtr xml_root_xps;	/* Pointer to the root node of the <xps> subtree	*/
    xmlNodePtr xml_node_xp;	/* Pointer to the children of <xps> */
    xmlNodePtr xml_root_apps;	/* Pointer to the root node of the <applications> subtree	*/
    xmlNodePtr xml_node_app;	/* Pointer to the children of <applications>	*/


    /* Are there any SWMs registered ?	*/
    dirp = opendir(CPM_SWMS_PATH);
    if (!readdir(dirp))
    {
        syslog(LOG_INFO, "SWMs directory empty\n");
        closedir(dirp);
        return E_NO_SWMS;
    }
    closedir(dirp);

    /* Parse the config file in a tree */
    doc = xmlParseFile(config_file_path);

    if (doc == NULL)
    {
        syslog(LOG_INFO, "Error in parsing configuration file %s\n", config_file_path);
        return E_CONFIG;
    }
    /* Get the root of the config file */
    cur = xmlDocGetRootElement(doc);

    if (cur == NULL)
    {
        syslog(LOG_INFO, "Error in parsing configuration file %s (empty)\n", config_file_path);
        xmlFreeDoc(doc);
        return E_CONFIG;
    }

    /* Validation of the config file	*/
    if (xmlStrcmp(cur->name, (const xmlChar *) "cpm-config"))
    {
        syslog(LOG_INFO, "Error in parsing configuration file %s (root node should be <cpm-config>)\n", config_file_path);
        xmlFreeDoc(doc);
        return E_CONFIG;
    }

    cur = cur->xmlChildrenNode;

    /* Step 1 : Let's the the eXecution Profiles defined in the config file	*/
    while ((cur) && (!xmlStrEqual(cur->name, (const xmlChar *) "xps")))
        cur = cur->next;

    while (cur != NULL)
    {
        if (!xmlStrcmp(cur->name, (const xmlChar *) "xps"))
        {
            /* Root of xps subtree	*/
            xml_root_xps = cur;
            xml_node_xp = cur->xmlChildrenNode;

            /* Validation of every xp declared */
            while (xml_node_xp != NULL)
            {
                if (!xmlStrcmp(xml_node_xp->name, (const xmlChar *) "xp"))
                    /* XP found... register it now! */
                    register_xp(xml_node_xp);

                xml_node_xp = xml_node_xp->next;
            }
        }

        /* Check if some XP have been registered	*/
        if (xps_list->count == 0)
        {
            syslog(LOG_WARNING, "No execution profiles registered.");
            xmlFreeDoc(doc);
            return E_LIST;
        }

        /* Step 2 : parse the applications data 	*/
        while ((cur) && (!xmlStrEqual(cur->name, (const xmlChar *) "applications")))
            cur = cur->next;

        if (!cur) break;

        if (!xmlStrcmp(cur->name, (const xmlChar *) "applications"))
        {
            print_debug( "Applications...\n");
            /* Root of the applications subtree	*/
            xml_root_apps = cur;
            xml_node_app = xml_root_apps->xmlChildrenNode;

            while (xml_node_app != NULL)
            {
                if (!xmlStrcmp(xml_node_app->name, (const xmlChar *) "app"))
                    /* Validation of the association application - xp	*/
                    register_application(xml_node_app);

                xml_node_app = xml_node_app->next;
            }
        }
        cur = cur->next;
    }

    print_debug("\n> %lu execution profiles registered.\n", xps_list->count);

    /* Check if some applications have been registered	*/
    if (applications_list->count == 0)
    {
        syslog(LOG_WARNING, "%s", "No application specified.");
        xmlFreeDoc(doc);
        return E_LIST;
    }
    else
        print_debug("> %lu applications registered.\n\n", applications_list->count);

    xmlFreeDoc(doc);
    return 0;
}


/**
 * register_xp()
 *
 * @brief For every xp check if all the system-wide metrics (SWM) are correctly specified.
 * @param xplist The list of xps
 * @param xml_node_xp The pointer to the root of xps subtree
 */
void register_xp(xmlNodePtr xml_node_xp)
{
    xmlNodePtr xml_node_swm;
    struct constraint_data *curr_constr = NULL;
    struct xp_data *new_xp = NULL;
    struct sysfs_attribute *sysfs_attr;
    char sysfs_swm_name[SYSFS_PATH_MAX];
    char sysfs_xp_swms[SYSFS_PATH_MAX];
    char *xml_str = NULL;
    char *swm_name = NULL;
    char name_str[CPM_NAME_LEN];
    char data_for_attribute[CPM_COMM_LEN];


    /* Set the path of eXecution Profiles	*/
    strcpy(sysfs_xp_swms, CPM_XPS_PATH);

    /* Create the new xp	*/
    new_xp = malloc(sizeof(struct xp_data));

    /* Set the name of the xp	*/
    xml_str = (char *) xmlGetProp(xml_node_xp, (const xmlChar *) "name");
    strncpy(new_xp->name, xml_str, sizeof(new_xp->name));
    print_debug("xp = %s : \n", new_xp->name);

    /* Initialize the list of SWMs 	*/
    new_xp->constraints = dlist_new(sizeof(struct constraint_data));

    /* Iterate on the SWMs specified by the current XP	*/
    xml_node_swm = xml_node_xp->xmlChildrenNode;

    while (!xmlStrcmp(xml_node_xp->name, (const xmlChar *) "swm"))
        xml_node_swm = xml_node_swm->next;

    while (xml_node_swm)
    {
        if ((swm_name = (char *) xmlGetProp(xml_node_swm, (const xmlChar *) "name")))	/* ex. "CPM_NETWORK_LATENCY" */
        {
            curr_constr = NULL;

            /* Build the path for the corrisponding sysfs attribute	*/
            strcpy(sysfs_swm_name, CPM_SWMS_PATH);
            strncat(sysfs_swm_name, swm_name, CPM_NAME_LEN);
            print_debug("\tswm = %s\n", sysfs_swm_name);

            /* Get the sysfs attribute relative to the SWM */
            if ((sysfs_attr = sysfs_open_attribute(sysfs_swm_name)))
            {
                curr_constr = malloc(sizeof(struct constraint_data));
                strncpy(curr_constr->swm_name, swm_name, sizeof(curr_constr->swm_name));

                /* Get the bound value requested for the SWM	*/
                xml_str = (char *) xmlGetProp(xml_node_swm, (const xmlChar *) "value");
                if (xml_str)
                    curr_constr->value = atoi(xml_str);
                else
                    curr_constr->value = 0;

                print_debug("\tConfig: swm=%s value=%d\n", curr_constr->swm_name, curr_constr->value);

                /* Get the data of SWM as exported in the sysfs	*/
                if (sysfs_read_attribute(sysfs_attr) == 0)
                    sscanf(sysfs_attr->value, "%u:%s %c %c %u %u %c %s", &curr_constr->swm_id, name_str, &curr_constr->type, &curr_constr->composition,
                           &curr_constr->min, &curr_constr->max, &curr_constr->permission, curr_constr->constraint);
                else
                {
                    syslog(LOG_ERR, "Cannot read from attribute %s", sysfs_swm_name);
                    return;
                }
                print_debug("\tPlatform: %s min=%u max=%d\n", name_str, curr_constr->min, curr_constr->max);

                /* Check the range request validity	*/
                if ((curr_constr->value >= curr_constr->min) && (curr_constr->value <= curr_constr->max))
                {
                    /* Insert the SWM in the list */
                    dlist_insert_sorted(new_xp->constraints, (struct constraint_data *) curr_constr, swm_name_compare);
                }
                else
                {
                    syslog(LOG_ERR, "Uncorrect range for SWM '%s'\n", swm_name);
                    print_debug("Uncorrect range for SWM '%s'\n", swm_name);
                    free(curr_constr);
                    dlist_destroy(new_xp->constraints);
                    return;
                }
            }
            else
                syslog(LOG_ERR, "SWM '%s' not found.\n", swm_name);

            sysfs_close_attribute(sysfs_attr);

            if (!curr_constr)
            {
                syslog(LOG_ERR, "XP '%s' not registered.\n", new_xp->name);
                free(new_xp);
                return;
            }
        }
        /* Let's see the next SWM...	*/
        xml_node_swm = xml_node_swm->next;
    }

    /* Signal the new XP to CPM*/
    memset(data_for_attribute, 0, sizeof(data_for_attribute));

    if ((sysfs_attr = sysfs_open_attribute(CPM_NEW_XP_PATH)))
    {
        snprintf(data_for_attribute, CPM_COMM_LEN, "%s %s ", control_comm[CTRL_ADD], new_xp->name);

        if (!sysfs_write_attribute(sysfs_attr, data_for_attribute, strlen(data_for_attribute)))
            syslog(LOG_INFO, "Profile %s signaled for '%s'\n", new_xp->name, control_comm[0]);
    }
    sysfs_close_attribute(sysfs_attr);

    /* Register the constraints of the XP	*/
    strncat(sysfs_xp_swms, new_xp->name, CPM_NAME_LEN);
    strcat(sysfs_xp_swms, "/constraints");
    memset(data_for_attribute, 0, sizeof(data_for_attribute));
    curr_constr = NULL;

    if ((sysfs_attr = sysfs_open_attribute(sysfs_xp_swms)))
    {
        dlist_for_each_data(new_xp->constraints, curr_constr, struct constraint_data)
        {
            snprintf(data_for_attribute, CPM_COMM_LEN, "%d %d ", curr_constr->swm_id, curr_constr->value);

            if (!sysfs_write_attribute(sysfs_attr, data_for_attribute, strlen(data_for_attribute)))
                syslog(LOG_INFO, "SWM info (ID VALUE) = %s\n", data_for_attribute);
        }
    }
    else
        syslog(LOG_ERR, "SWM path '%s' not found\n.", sysfs_xp_swms);

    sysfs_close_attribute(sysfs_attr);

    /* Insert the XP in the list	*/
    dlist_insert_sorted(xps_list, (struct xp_data *) new_xp, xp_name_compare);
    syslog(LOG_INFO, "XP '%s' registered.\n", new_xp->name);
    print_debug("%s registered.\n\n", new_xp->name);
}


/**
 * register_application()
 *
 * @brief For every application check if the xp specified exists
 * @param xml_node_app : The current xml "application" node
 */
void register_application(xmlNodePtr xml_node_app)
{
    xmlNodePtr xml_node_xp_text;
    struct xp_data *xp_node	= NULL;
    struct app_data *new_app = NULL;
    char *xml_str = NULL;
    char *xps_buffer = NULL;
    char single_xp[CPM_NAME_LEN];
    int c_count = 0;
    int buf_size;

    /* Retrieve the execution profiles associated to the application	*/
    xml_node_xp_text = xml_node_app->xmlChildrenNode;

    /* Create a new application object	*/
    new_app = malloc(sizeof(struct app_data));

    /* Init the list of XPs	*/
    new_app->xps = dlist_new(sizeof (struct xp_data));
    //struct dlist *xpl = new_app->xps;

    /* Set the name of the application */
    xml_str = (char *) xmlGetProp(xml_node_app, (const xmlChar *) "name");
    if (xml_str)
        strncpy(new_app->name, xml_str, sizeof(new_app->name));
    else
    {
        free(new_app);
        return;
    }
    /* Set the priority	*/
    xml_str = (char *) xmlGetProp(xml_node_app, (const xmlChar *) "priority");
    if (xml_str)
        new_app->priority = atoi(xml_str);
    else
        new_app->priority = default_prog_prio;

    /* Init the list of PIDs	*/
    new_app->pid_instances = dlist_new(sizeof (struct pid_data));

    /* Retrieve all the XPs specified in the config file	*/
    buf_size = min(strlen((char *) xml_node_xp_text->content) + 1, BUF_MAX_DIM_ALLOW);
    xps_buffer = (char *) malloc(buf_size);
    memset(xps_buffer, 0, sizeof(xps_buffer));

    strncpy(xps_buffer, (char *) xml_node_xp_text->content, buf_size);
    memset(single_xp, 0, sizeof(single_xp));

    while (c_count < strlen((char *) xml_node_xp_text->content))
    {
        sscanf(xps_buffer, "%s", single_xp);
        /* Check if the execution profile exists	*/
        xp_node = xp_registered(single_xp);

        if (xp_node)
        { 	/* Place it in the list	new_app->xps*/
            dlist_insert_sorted(new_app->xps, (struct xp_data *) xp_node, xp_name_compare);
            print_debug("\t'%s' will assert (%lu) %s\n", new_app->name, new_app->xps->count, single_xp);
        }
        else
            print_debug("\t'%s' not found in xp list\n", single_xp);

        c_count += (strlen(single_xp) + 1);
        xps_buffer += c_count;
    }

    /* No valid profiles have been specified	*/
    if (new_app->xps->count == 0)
    {
        dlist_destroy(new_app->xps);
        dlist_destroy(new_app->pid_instances);
        free(new_app);
        return;
    }

    /* Insert the new application object in applications_list*/
    dlist_insert_sorted(applications_list, (struct app_data *) new_app, app_name_compare);
}

/**
 * xp_registered()
 *
 * @brief Check if the xp specified with "xp_name" has been registered
 * @param xp_name The name of the xp to find
 *
 * @return The pointer to the xp data structure found, NULL otherwise.
 */
struct xp_data *xp_registered(char *xp_name)
{
    struct xp_data target_xp;

    target_xp.constraints = NULL;

    if (!xps_list) return NULL;
    /* Set the target object for the search	*/
    strncpy(target_xp.name, xp_name, sizeof(target_xp.name));

    /* Return if found the XP data object from the list	*/
    return (struct xp_data *) dlist_find_custom(xps_list, (struct xp_data *) &target_xp, xp_name_match);
}

/**
 * get_app_by_name ()
 *
 * @brief Given the application name try to look up the application running profile
 * @param app_name The application name
 * @return The reference to the struct app_data to return.
 */
struct app_data *get_app_by_name(char *app_name)
{
    assert(applications_list != NULL);
    assert(xps_list != NULL);

    struct app_data target_appl;

    if (!app_name) return NULL;

    /* Set the target object for the search	*/
    strncpy(target_appl.name, app_name, sizeof(target_appl.name));
    /* Find the application data object in the list	*/
    return (struct app_data *) dlist_find_custom(applications_list, (struct app_data *) &target_appl, app_name_match);
}

/**
 * xp_assert()
 *
 * @brief Write on the sysfs interface the application running profile to do or undo
 * @param application_info The app_data struct with the infos upon the application asserting the profile
 * @param pids - PIDs of the process instance of the application
 * @param command - The command for the assertion
 */
void assert_xp(struct app_data *application_info, struct pid_data *pids, int comm_index)
{
    struct sysfs_attribute *attr;
    struct xp_data *xp_p = NULL;
    char assertion[CPM_COMM_LEN];

    if (!application_info) return;

    /* Open the sysfs attribute '.../xps/assert'	*/
    attr = sysfs_open_attribute(CPM_ASSERT_XP_PATH);

    if (!attr)
    {
        syslog(LOG_ERR, "Assertion not sent (%s)", assertion);
        sysfs_close_attribute(attr);
        return;
    }

    dlist_for_each_data(application_info->xps, xp_p, struct xp_data)
    {
        /* Build the XP assertion string	*/
        snprintf(assertion, CPM_COMM_LEN, "%s %s %d %d %s %d", assert_comm[comm_index], xp_p->name, pids->pid, pids->tgid,
                 application_info->name, application_info->priority);

        /* Assert the profile	*/
        if (sysfs_write_attribute(attr, assertion, strlen(assertion)) == 0)
            syslog(LOG_INFO, "Assertion OK (%s)", assertion);
    }

    sysfs_close_attribute(attr);
}


/**
 * get_app_by_pid()
 *
 * @param pid
 * @param tgid
 *
 * @return The struct app_data if an application with pid and tgid has found in the list
 */
struct app_data *get_app_by_pid(struct pid_data * target_pid)
{
    struct app_data *ad = NULL;

    dlist_for_each_data(applications_list, ad, struct app_data)
    {
        if (ad->pid_instances->count > 0)
            if (dlist_find_custom(ad->pid_instances, (struct pid_data *) target_pid, app_pid_compare))
                return ad;
    }
    return NULL;
}


/**
 * handle_intr()
 *
 * @brief Handle the SIGINT signal for a graceful exit
 */
void handle_intr(int signum)
{
    siglongjmp(g_jmp, signum);
    dlist_destroy(xps_list);
    dlist_destroy(applications_list);
    syslog(LOG_INFO, "Terminated.");
    closelog();
}

/********************************************************
 *  		UTILITY FUNCTIONS			*
 * ******************************************************/


/**
 * app_pid_compare()
 *
 * @brief Just return the compare result of the pid of two application
 * @param first The first pid value
 * @param second The second pid value
 *
 * @return 1 if the pid are equals, 0 otherwise.
 */
int app_pid_compare(void *first, void *second)
{
    if (((((struct pid_data *)first)->pid == ((struct pid_data *)second)->pid))
            && (((struct pid_data *)first)->tgid == ((struct pid_data *)second)->tgid))
        return 1;
    else
        return 0;
}

/**
 * swm_name_compare()
 *
 * @brief Just return the compare result of the name of the SWMs for dlist order purpose
 * @param first The first SMW
 * @param second The second SWM
 *
 * @return What strcmp() return.
 */
int swm_name_compare(void *first, void *second)
{
    return strcmp(((struct constraint_data *)first)->swm_name, ((struct constraint_data *)second)->swm_name);
}

/**
 * xp_name_compare()
 *
 * @brief Just return the compare result of the name of the XPs for dlist order purpose
 * @param first The first XP
 * @param second The second XP
 *
 * @return What strcmp() return.
 */
int xp_name_compare(void *first, void *second)
{
    return strcmp(((struct xp_data *)first)->name, ((struct xp_data *)second)->name);
}

/**
 * xp_name_match()
 *
 * @brief The search function for the XP list.
 * @return It calls xp_name_compare() returning the reverse result.
 */
int xp_name_match(void *first, void *second)
{
    return !xp_name_compare(first, second);
}

/**
 * app_name_compare()
 *
 * @brief Just return the compare result of the name of the applications for dlist order purpose
 * @param first The first app_data structure
 * @param second The second app_data structure
 *
 * @return What strcmp() return.
 */
int app_name_compare(void *first, void *second)
{
    return strcmp(((struct app_data *)first)->name, ((struct app_data *)second)->name);
}

/**
 * app_name_match()
 *
 * @brief The search function for the applications list.
 * @return It calls app_name_compare() returning the reverse result.
 */
int app_name_match(void *first, void *second)
{
    return !app_name_compare(first, second);
}

/**
 * print_help()
 *
 * @brief Print command-line usage.
 */
void print_help()
{
    fprintf(stderr, "\n# ./cpm_daemon -c 'config_file_path' [options] \n"
            "\t (The daemon must be start as root)\n"
            "OPTIONS :\n"
            "\t-c 'filepath' \tSpecify the xml configuration file \n"
            "\t-p num \tDefault priority value for applications\n"
            "\t-d \tRun in debug mode\n"
            "\t-v \tPrint version informations\n"
            "\t-h \tPrint help usage\n\n"
           );
}
