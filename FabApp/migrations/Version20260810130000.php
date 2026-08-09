<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/** S127: portal retirement for the disposable Artemis development dataset. */
final class Version20260810130000 extends AbstractMigration
{
    public function isTransactional(): bool { return false; }
    public function getDescription(): string { return 'Retire the unused portal scope after an empty preflight report.'; }

    public function up(Schema $schema): void
    {
        // Artemis is an explicitly disposable development instance. Drop overrides
        // rather than preserving multi-site state, but retain packages as the one
        // site's package catalogue.
        $this->addSql('DELETE FROM SITE_SETTING WHERE portalId <> 0');
        $this->addSql('DELETE FROM SITE_MODULE WHERE portalId <> 0');
        $this->addSql('DELETE FROM MISSING_PAGE WHERE portalId <> 0');
        $this->addSql('UPDATE USAGE_PACKAGE SET portalId = 0 WHERE portalId <> 0');

        $this->addSql('ALTER TABLE SITE_SETTING DROP PRIMARY KEY, DROP COLUMN portalId, ADD PRIMARY KEY (settingKey)');
        $this->addSql('ALTER TABLE SITE_MODULE DROP PRIMARY KEY, DROP COLUMN portalId, ADD PRIMARY KEY (moduleKey)');
        $this->addSql('ALTER TABLE USAGE_PACKAGE DROP INDEX IDX_USAGE_PACKAGE_PORTAL, DROP COLUMN portalId');
        $this->addSql('ALTER TABLE EMAIL_LOG DROP COLUMN portalId');
        $this->addSql('ALTER TABLE MISSING_PAGE DROP INDEX UNIQ_MISSING_PAGE_PATH, DROP COLUMN portalId, ADD UNIQUE INDEX UNIQ_MISSING_PAGE_PATH (path)');
        $this->addSql('DROP TABLE PORTAL');
    }

    public function down(Schema $schema): void
    {
        throw new \RuntimeException('S127 is restored from the verified pre-migration backup; do not synthesize portal data on rollback.');
    }
}
