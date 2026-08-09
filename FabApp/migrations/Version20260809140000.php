<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

final class Version20260809140000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'Add stable protected built-in group metadata without changing current role security semantics.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql('ALTER TABLE ROLE ADD groupKey VARCHAR(50) DEFAULT NULL, ADD label VARCHAR(100) DEFAULT NULL, ADD description VARCHAR(255) DEFAULT NULL, ADD isProtected TINYINT(1) NOT NULL DEFAULT 0');
        $this->addSql("UPDATE ROLE SET groupKey = CASE nom WHEN 'admin' THEN 'admin' WHEN 'staff' THEN 'staff' WHEN 'user' THEN 'user' WHEN 'trainer' THEN 'trainers' END, label = CASE nom WHEN 'admin' THEN 'Administrateur global' WHEN 'staff' THEN 'Staff' WHEN 'user' THEN 'User' WHEN 'trainer' THEN 'Formateurs' END, description = CASE nom WHEN 'admin' THEN 'Récupération système hors packages.' WHEN 'staff' THEN 'Équipe opérationnelle.' WHEN 'user' THEN 'Audience de tout compte actif.' WHEN 'trainer' THEN 'Formateurs du FabOS.' END, isProtected = 1 WHERE nom IN ('admin', 'staff', 'user', 'trainer')");
        $this->addSql("INSERT INTO ROLE (nom, groupKey, label, description, isProtected) VALUES ('manager', 'manager', 'Manager', 'Responsable métier.', 1), ('super_user', 'super_user', 'Super user', 'Utilisateur avancé.', 1), ('guest', 'guest', 'Guest', 'Audience anonyme virtuelle.', 1)");
        $this->addSql('ALTER TABLE ROLE ADD UNIQUE INDEX UNIQ_ROLE_GROUP_KEY (groupKey)');
    }

    public function down(Schema $schema): void
    {
        $this->addSql("DELETE FROM ROLE WHERE groupKey IN ('manager', 'super_user', 'guest') AND isProtected = 1");
        $this->addSql('ALTER TABLE ROLE DROP INDEX UNIQ_ROLE_GROUP_KEY, DROP groupKey, DROP label, DROP description, DROP isProtected');
    }
}
